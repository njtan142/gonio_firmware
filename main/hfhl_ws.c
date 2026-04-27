#include "hfhl_ws.h"
#include "app_config.h"
#include "mt6701.h"
#include "ring_buffer.h"
#if !ENABLE_WEBSERVER_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>

static const char *TAG = "hfhl_ws";

/* Battery SoC%, defined in main.c, updated by ui_task. */
extern float soc_percent;

static httpd_handle_t s_server = NULL;
static volatile int s_client_fd = -1;
static volatile bool s_acq_run = false;
static volatile int s_acq_fpp = 1;      /* frames per batch  (1–46) */
static volatile int s_acq_freq_hz = 60; /* batch frequency   (Hz)   */

/* ── Frame packing ──────────────────────────────────────────────────────────
 * Same 8-byte big-endian layout as webrtc.c:
 *   [63:50] 14-bit angle  [49:18] 32-bit µs timestamp
 *   [17:10]  8-bit SoC    [ 9: 0] 10-bit status flags
 */
static void pack_frame(uint8_t *buf, float deg, int64_t t_us, uint16_t flags) {
  uint16_t raw = (uint16_t)(deg * (16384.0f / 360.0f)) & 0x3FFF;
  uint32_t ts = (uint32_t)(t_us & 0xFFFFFFFFULL);
  uint8_t soc = (uint8_t)(soc_percent * 2.55f);
  uint64_t frame = ((uint64_t)raw << 50) | ((uint64_t)ts << 18) |
                   ((uint64_t)soc << 10) | ((uint64_t)(flags & 0x3FF));
  for (int i = 7; i >= 0; i--) {
    buf[i] = (uint8_t)(frame & 0xFF);
    frame >>= 8;
  }
}

/* ── Static send buffer + semaphore ─────────────────────────────────────────
 * One fixed BSS buffer shared across all sends.  The binary semaphore ensures
 * ws_send_cb has finished with the buffer before the flush task refills it.
 * ws_job_t is a tiny header-only struct — no per-send heap allocation for payload.
 */
#define WS_SEND_SAMPLES 512  /* send buffer chunk — flush loop drains remaining */

/* Both buffers are heap-allocated on WS connect and freed on disconnect so the
 * DTLS context can reclaim RAM during WebRTC sessions (§7.5). */
static uint8_t          *s_send_buf     = NULL; /* WS_SEND_SAMPLES × RB_SAMPLE_SIZE bytes */
static uint8_t          *s_compress_buf = NULL; /* 4096 bytes: v2 compressed output */
static SemaphoreHandle_t s_send_sem; /* 1 = buffer free, 0 = send in flight */

typedef struct {
  httpd_handle_t server;
  int fd;
  size_t len;
  uint8_t *data; /* points into s_compress_buf — NOT owned by this job */
} ws_job_t;

static void ws_send_cb(void *arg) {
  ws_job_t *job = (ws_job_t *)arg;
  httpd_ws_frame_t frame = {
      .final = true,
      .fragmented = false,
      .type = HTTPD_WS_TYPE_BINARY,
      .payload = job->data,
      .len = job->len,
  };
  if (httpd_ws_send_frame_async(job->server, job->fd, &frame) != ESP_OK) {
    s_acq_run = false;
    s_client_fd = -1;
    rb_free();
    /* Give semaphore before freeing so ws_flush_task exits its NULL guard cleanly */
    xSemaphoreGive(s_send_sem);
    free(s_send_buf);     s_send_buf     = NULL;
    free(s_compress_buf); s_compress_buf = NULL;
    ESP_LOGW(TAG, "ws send failed — client fd=%d dropped", job->fd);
    free(job);
    return;
  }
  xSemaphoreGive(s_send_sem); /* buffer is free to refill */
  free(job);
}

/* ── FreeRTOS tasks ─────────────────────────────────────────────────────────
 *
 * sensor_acq_task  — high priority producer:
 *   Reads all 4 MT6701 sensors at s_acq_hz, packs one 32-byte sample, and
 *   pushes it into the ring buffer.  Runs whenever s_acq_run is true,
 *   regardless of whether a WebSocket client is connected.
 *
 * ws_flush_task — low priority consumer:
 *   Drains the ring buffer in FLUSH_BATCH-sample chunks and queues each chunk
 *   for delivery to the connected WebSocket client via httpd_queue_work.
 *   Sleeps when no client is connected or when the buffer is empty.
 */

/* Decode the 14-bit angle from a sensor slot within a packed 32-byte sample. */
static float decode_angle(const uint8_t *sample, int sensor) {
  const uint8_t *b = sample + sensor * 8;
  uint64_t frame = 0;
  for (int i = 0; i < 8; i++) frame = (frame << 8) | b[i];
  uint16_t raw = (uint16_t)((frame >> 50) & 0x3FFF);
  return raw * (360.0f / 16384.0f);
}

/* ── v2 batch compression ────────────────────────────────────────────────────
 * Wire format (v2):
 *   Header 18 B: [0]=2, [1-2]=sample_count LE, [3-6]=base_ts LE,
 *                [7-14]=4×uint16_t base_angles LE, [15]=soc, [16-17]=flags LE
 *   Per delta (6 B per sample 1..n-1):
 *                [0-3]=4×int8_t delta_angle, [4-5]=uint16_t delta_ts LE
 * Returns number of bytes written to out (≤ 4096 for n ≤ 512). */
static int compress_batch(const uint8_t *raw, int n, uint8_t *out) {
  if (n == 0) return 0;

  /* Decode first sample — sensor 0 carries ts/soc/flags shared across all 4 */
  uint64_t f0 = 0;
  for (int i = 0; i < 8; i++) f0 = (f0 << 8) | raw[i];
  uint32_t base_ts = (uint32_t)((f0 >> 18) & 0xFFFFFFFFULL);
  uint8_t  soc     = (uint8_t)((f0 >> 10) & 0xFF);
  uint16_t flags   = (uint16_t)(f0 & 0x3FF);

  uint16_t prev_ang[4];
  for (int s = 0; s < 4; s++) {
    uint64_t sf = 0;
    const uint8_t *b = raw + s * 8;
    for (int i = 0; i < 8; i++) sf = (sf << 8) | b[i];
    prev_ang[s] = (uint16_t)((sf >> 50) & 0x3FFF);
  }

  /* Header */
  out[0] = 2;
  out[1] = (uint8_t)(n        & 0xFF);
  out[2] = (uint8_t)((n >> 8) & 0xFF);
  out[3] = (uint8_t)(base_ts         & 0xFF);
  out[4] = (uint8_t)((base_ts >>  8) & 0xFF);
  out[5] = (uint8_t)((base_ts >> 16) & 0xFF);
  out[6] = (uint8_t)((base_ts >> 24) & 0xFF);
  for (int s = 0; s < 4; s++) {
    out[7 + s * 2]     = (uint8_t)( prev_ang[s]       & 0xFF);
    out[7 + s * 2 + 1] = (uint8_t)((prev_ang[s] >> 8) & 0xFF);
  }
  out[15] = soc;
  out[16] = (uint8_t)( flags       & 0xFF);
  out[17] = (uint8_t)((flags >> 8) & 0xFF);

  int pos = 18;
  uint32_t prev_ts = base_ts;

  for (int t = 1; t < n; t++) {
    const uint8_t *sample = raw + (size_t)t * RB_SAMPLE_SIZE;

    uint64_t sf0 = 0;
    for (int i = 0; i < 8; i++) sf0 = (sf0 << 8) | sample[i];
    uint32_t curr_ts  = (uint32_t)((sf0 >> 18) & 0xFFFFFFFFULL);
    uint16_t delta_ts = (uint16_t)((curr_ts - prev_ts) & 0xFFFF);
    prev_ts = curr_ts;

    for (int s = 0; s < 4; s++) {
      uint64_t sf = 0;
      const uint8_t *b = sample + s * 8;
      for (int i = 0; i < 8; i++) sf = (sf << 8) | b[i];
      uint16_t curr_ang = (uint16_t)((sf >> 50) & 0x3FFF);
      int d = (int)curr_ang - (int)prev_ang[s];
      if (d >  8192) d -= 16384;  /* wrap-around toward 0 */
      if (d < -8192) d += 16384;
      if (d >  127)  d =  127;    /* clamp — only hits if joint moves >2.79°/sample */
      if (d < -128)  d = -128;
      out[pos + s] = (uint8_t)(int8_t)d;
      prev_ang[s] = curr_ang;
    }
    out[pos + 4] = (uint8_t)( delta_ts       & 0xFF);
    out[pos + 5] = (uint8_t)((delta_ts >> 8) & 0xFF);
    pos += 6;
  }
  return pos;
}

static void ws_flush_task(void *pv) {
  uint32_t stat_samples = 0;
  uint32_t stat_bytes   = 0;
  uint32_t stat_qfails  = 0;
  int64_t  stat_next_us = esp_timer_get_time() + 1000000;

  while (1) {
    int fd = s_client_fd;
    if (fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      stat_next_us = esp_timer_get_time() + 1000000;
      continue;
    }

    if (rb_count() == 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      goto maybe_log;
    }

    /* Wait for the previous send to finish before refilling s_send_buf.
     * ws_send_cb gives this semaphore after the TCP send completes. */
    xSemaphoreTake(s_send_sem, portMAX_DELAY);

    fd = s_client_fd; /* re-check after potentially sleeping on semaphore */
    if (fd < 0) {
      xSemaphoreGive(s_send_sem);
      goto maybe_log;
    }

    if (!s_send_buf || !s_compress_buf) {
      xSemaphoreGive(s_send_sem);
      vTaskDelay(pdMS_TO_TICKS(100));
      goto maybe_log;
    }

    int available = rb_count();
    int n = (available < WS_SEND_SAMPLES) ? available : WS_SEND_SAMPLES;
    int popped = rb_pop_batch(s_send_buf, n);
    if (popped == 0) {
      xSemaphoreGive(s_send_sem);
      goto maybe_log;
    }

    int compressed_len = compress_batch(s_send_buf, popped, s_compress_buf);

    ws_job_t *job = malloc(sizeof(ws_job_t)); /* tiny header only — always succeeds */
    if (!job) {
      xSemaphoreGive(s_send_sem);
      vTaskDelay(pdMS_TO_TICKS(100));
      goto maybe_log;
    }
    job->server  = s_server;
    job->fd      = fd;
    job->data    = s_compress_buf;
    job->len     = (size_t)compressed_len;
    size_t bytes = job->len; /* snapshot before hand-off — ws_send_cb frees job */

#if ENABLE_WS_DATA_TRACE
    ESP_LOGI(TAG, "flush n=%d  s0=%.2f° s1=%.2f° s2=%.2f° s3=%.2f°",
             popped,
             decode_angle(s_send_buf, 0),
             decode_angle(s_send_buf, 1),
             decode_angle(s_send_buf, 2),
             decode_angle(s_send_buf, 3));
#endif

    if (httpd_queue_work(s_server, ws_send_cb, job) != ESP_OK) {
      /* Queue full — give semaphore back (buffer not in use) and back off. */
      free(job);
      xSemaphoreGive(s_send_sem);
      stat_qfails++;
      vTaskDelay(pdMS_TO_TICKS(15));
    } else {
      stat_samples += (uint32_t)popped;
      stat_bytes   += (uint32_t)bytes;
      /* semaphore is now held by the in-flight ws_send_cb */
    }

maybe_log:
    if (esp_timer_get_time() >= stat_next_us) {
      int drops = rb_get_drops();
      rb_reset_drops();
      ESP_LOGI(TAG, "WS stats 1s: sent=%lu smp (%lu B)  rb_drops=%d  q_fails=%lu  rb_now=%d  free_heap=%u",
               (unsigned long)stat_samples,
               (unsigned long)stat_bytes,
               drops,
               (unsigned long)stat_qfails,
               rb_count(),
               (unsigned)esp_get_free_heap_size());
      stat_samples = 0;
      stat_bytes   = 0;
      stat_qfails  = 0;
      stat_next_us = esp_timer_get_time() + 1000000;
    }
  }
}

static void sensor_acq_task(void *pv) {
  static uint8_t sample[RB_SAMPLE_SIZE];

  uint32_t stat_produced  = 0;
  uint32_t stat_err_flags = 0; /* bitmask of sensors that fired an error this window */
  float    stat_min[4]    = {360.0f, 360.0f, 360.0f, 360.0f};
  float    stat_max[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
  int64_t  stat_next_us   = esp_timer_get_time() + 1000000;

  while (1) {
    if (!s_acq_run) {
      vTaskDelay(pdMS_TO_TICKS(100));
      stat_next_us = esp_timer_get_time() + 1000000;
      continue;
    }

    /* Snapshot batch parameters atomically */
    int fpp = s_acq_fpp;
    int freq_hz = s_acq_freq_hz;
    int64_t batch_start_us = esp_timer_get_time();

    /* Read fpp samples back-to-back in a tight loop — each gets its
     * own hardware timestamp so the receiver can recover real timing.
     * Hold the SPI bus for the entire batch so spi_device_polling_transmit
     * skips the per-transaction mutex acquire/release (~43 µs overhead each).
     * Without this, 46 separate acquires dominate the 2,500 µs batch window.
     * HFHL: block when the ring buffer is full rather than dropping. */
    mt6701_acquire_bus();
    for (int f = 0; f < fpp && s_acq_run; f++) {
      while (s_acq_run && rb_count() >= RB_CAPACITY) {
        mt6701_release_bus();
        vTaskDelay(1); /* yield until consumer frees space */
        mt6701_acquire_bus();
      }
      if (!s_acq_run) break;

      int64_t now = esp_timer_get_time();
      float deg[4];
      uint16_t flags = 0;
      for (int i = 0; i < 4; i++) {
        deg[i] = mt6701_get_degrees_fast(i);
        if (mt6701_has_error(i)) {
          flags |= (uint16_t)(1u << (9 - i));
          stat_err_flags |= (1u << i);
        }
        if (deg[i] < stat_min[i]) stat_min[i] = deg[i];
        if (deg[i] > stat_max[i]) stat_max[i] = deg[i];
      }
      for (int i = 0; i < 4; i++) {
        pack_frame(&sample[i * 8], deg[i], now, flags);
      }
      rb_push(sample);
      stat_produced++;
    }
    mt6701_release_bus();

    /* Wait for the next batch period.  On single-core ESP32-C3,
     * we MUST call vTaskDelay (not taskYIELD) so the lower-priority
     * ws_flush_task gets CPU time to drain the ring buffer.
     * Minimum 1 tick ensures the flush task always runs. */
    int64_t batch_period_us = 1000000 / freq_hz;
    int64_t elapsed_us = esp_timer_get_time() - batch_start_us;
    int64_t remaining_us = batch_period_us - elapsed_us;

    TickType_t ticks =
        (remaining_us > 1000) ? pdMS_TO_TICKS(remaining_us / 1000) : 1;
    vTaskDelay(ticks > 0 ? ticks : 1);

    if (esp_timer_get_time() >= stat_next_us) {
      ESP_LOGI(TAG,
               "ACQ stats 1s: produced=%lu  err_mask=0x%lx"
               "  s0=[%.1f,%.1f] s1=[%.1f,%.1f] s2=[%.1f,%.1f] s3=[%.1f,%.1f]",
               (unsigned long)stat_produced,
               (unsigned long)stat_err_flags,
               stat_min[0], stat_max[0],
               stat_min[1], stat_max[1],
               stat_min[2], stat_max[2],
               stat_min[3], stat_max[3]);
      stat_produced  = 0;
      stat_err_flags = 0;
      for (int i = 0; i < 4; i++) { stat_min[i] = 360.0f; stat_max[i] = 0.0f; }
      stat_next_us += 1000000;
    }
  }
}

/* ── WebSocket URI handler ──────────────────────────────────────────────────
 * ESP-IDF calls this handler with HTTP_GET on the initial WebSocket upgrade,
 * then again for every subsequent data frame received from the client.
 * Control frames (PING/PONG/CLOSE) are handled automatically by ESP-IDF
 * because handle_ws_control_frames is false.
 */
static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    if (!rb_alloc()) {
      ESP_LOGE(TAG, "OOM — cannot allocate ring buffer for WS client");
      return ESP_ERR_NO_MEM;
    }
    s_send_buf     = malloc((size_t)WS_SEND_SAMPLES * RB_SAMPLE_SIZE);
    s_compress_buf = malloc(4096);
    if (!s_send_buf || !s_compress_buf) {
      ESP_LOGE(TAG, "OOM — cannot allocate send buffers for WS client");
      free(s_send_buf);     s_send_buf     = NULL;
      free(s_compress_buf); s_compress_buf = NULL;
      rb_free();
      return ESP_ERR_NO_MEM;
    }
    s_client_fd = httpd_req_to_sockfd(req);
    rb_reset();
    s_acq_run = true;
    ESP_LOGI(TAG, "WS client connected fd=%d — acquisition started (heap free: %u)",
             s_client_fd, (unsigned)esp_get_free_heap_size());
    return ESP_OK;
  }
  /* Drain any data frame the client sends — firmware does not parse them. */
  httpd_ws_frame_t pkt = {0};
  pkt.type = HTTPD_WS_TYPE_BINARY;
  esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
  if (ret != ESP_OK) {
    s_acq_run = false;
    s_client_fd = -1;
    rb_free();
    free(s_send_buf);     s_send_buf     = NULL;
    free(s_compress_buf); s_compress_buf = NULL;
    return ret;
  }
  if (pkt.len > 0) {
    uint8_t *buf = malloc(pkt.len);
    if (buf) {
      pkt.payload = buf;
      httpd_ws_recv_frame(req, &pkt, pkt.len);
      free(buf);
    }
  }
  return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void hfhl_ws_init(httpd_handle_t server) {
  s_server   = server;
  s_send_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(s_send_sem); /* initially free */
  rb_init(); /* mutex only — buffer stays on heap until a client connects */
  httpd_uri_t uri = {
      .uri = "/ws",
      .method = HTTP_GET,
      .handler = ws_handler,
      .is_websocket = true,
      .handle_ws_control_frames = false,
  };
  httpd_register_uri_handler(server, &uri);
  /* sensor_acq_task at same priority as peer_main_task so acquisitions
   * are not starved; ws_flush_task at low priority (it blocks on network). */
  xTaskCreate(sensor_acq_task, "hfhl_acq", 3072, NULL, 6, NULL);
  xTaskCreate(ws_flush_task, "hfhl_flush", 4096, NULL, 4, NULL);
  ESP_LOGI(TAG, "HFHL WebSocket ready at ws://192.168.4.1/ws");
}

void hfhl_ws_set_batch_params(int fpp, int freq_hz) {
  if (fpp >= 1 && fpp <= 46)
    s_acq_fpp = fpp;
  if (freq_hz >= 1 && freq_hz <= 1000)
    s_acq_freq_hz = freq_hz;
  ESP_LOGI(TAG, "HFHL batch params: fpp=%d freq=%dHz -> total=%dHz",
           s_acq_fpp, s_acq_freq_hz, s_acq_fpp * s_acq_freq_hz);
}

void hfhl_ws_stop(void) {
  s_acq_run = false;
  s_client_fd = -1;
  rb_free();
  /* Wait for any in-flight ws_send_cb to finish before freeing its buffer */
  xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(500));
  xSemaphoreGive(s_send_sem);
  free(s_send_buf);     s_send_buf     = NULL;
  free(s_compress_buf); s_compress_buf = NULL;
  ESP_LOGI(TAG, "HFHL acquisition stopped (heap free: %u)", (unsigned)esp_get_free_heap_size());
}

bool hfhl_ws_is_connected(void) { return s_client_fd >= 0; }
