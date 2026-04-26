#include "hfhl_ws.h"
#include "ring_buffer.h"
#include "mt6701.h"
#include "app_config.h"
#if !ENABLE_WEBSERVER_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "hfhl_ws";

/* Battery SoC%, defined in main.c, updated by ui_task. */
extern float soc_percent;

static httpd_handle_t s_server    = NULL;
static volatile int   s_client_fd = -1;
static volatile bool  s_acq_run      = false;
static volatile int   s_acq_fpp      = 1;     /* frames per batch  (1–46) */
static volatile int   s_acq_freq_hz  = 60;    /* batch frequency   (Hz)   */

/* ── Frame packing ──────────────────────────────────────────────────────────
 * Same 8-byte big-endian layout as webrtc.c:
 *   [63:50] 14-bit angle  [49:18] 32-bit µs timestamp
 *   [17:10]  8-bit SoC    [ 9: 0] 10-bit status flags
 */
static void pack_frame(uint8_t *buf, float deg, int64_t t_us, uint16_t flags) {
    uint16_t raw   = (uint16_t)(deg * (16384.0f / 360.0f)) & 0x3FFF;
    uint32_t ts    = (uint32_t)(t_us & 0xFFFFFFFFULL);
    uint8_t  soc   = (uint8_t)(soc_percent * 2.55f);
    uint64_t frame = ((uint64_t)raw  << 50)
                   | ((uint64_t)ts   << 18)
                   | ((uint64_t)soc  << 10)
                   | ((uint64_t)(flags & 0x3FF));
    for (int i = 7; i >= 0; i--) {
        buf[i] = (uint8_t)(frame & 0xFF);
        frame >>= 8;
    }
}

/* ── httpd_queue_work payload ───────────────────────────────────────────────
 * Heap-allocated job that carries the binary payload into the httpd task.
 * The callback frees it after sending.
 */
typedef struct {
    httpd_handle_t server;
    int            fd;
    size_t         len;
    uint8_t        data[];   /* flexible array — payload follows the header */
} ws_job_t;

static void ws_send_cb(void *arg) {
    ws_job_t *job = (ws_job_t *)arg;
    httpd_ws_frame_t frame = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_BINARY,
        .payload    = job->data,
        .len        = job->len,
    };
    if (httpd_ws_send_frame_async(job->server, job->fd, &frame) != ESP_OK) {
        /* Send failure — client disconnected. Stop acquisition, clear fd,
         * and free the ring buffer so WebRTC can reclaim the ~80 KB heap. */
        s_acq_run   = false;
        s_client_fd = -1;
        rb_free();
        ESP_LOGW(TAG, "ws send failed — client fd=%d dropped", job->fd);
    }
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

#define FLUSH_BATCH 32   /* samples per WebSocket frame (32 × 32 = 1 KB) */

static void ws_flush_task(void *pv) {
    static uint8_t s_flush_buf[FLUSH_BATCH * RB_SAMPLE_SIZE];

    while (1) {
        int fd = s_client_fd;
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int n = rb_pop_batch(s_flush_buf, FLUSH_BATCH);
        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        size_t payload_sz = (size_t)n * RB_SAMPLE_SIZE;
        ws_job_t *job = malloc(sizeof(ws_job_t) + payload_sz);
        if (!job) {
            ESP_LOGW(TAG, "OOM — discarding %d samples", n);
            continue;
        }
        job->server = s_server;
        job->fd     = fd;
        job->len    = payload_sz;
        memcpy(job->data, s_flush_buf, payload_sz);
        if (httpd_queue_work(s_server, ws_send_cb, job) != ESP_OK) {
            free(job);
        }
    }
}

static void sensor_acq_task(void *pv) {
    static uint8_t sample[RB_SAMPLE_SIZE];

    while (1) {
        if (!s_acq_run) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Snapshot batch parameters atomically */
        int fpp     = s_acq_fpp;
        int freq_hz = s_acq_freq_hz;
        int64_t batch_start_us = esp_timer_get_time();

        /* Read fpp samples back-to-back in a tight loop — each gets its
         * own hardware timestamp so the receiver can recover real timing. */
        for (int f = 0; f < fpp; f++) {
            int64_t  now   = esp_timer_get_time();
            float    deg[4];
            uint16_t flags = 0;
            for (int i = 0; i < 4; i++) {
                deg[i] = mt6701_get_degrees(i);
                if (mt6701_has_error(i)) flags |= (uint16_t)(1u << (9 - i));
            }
            for (int i = 0; i < 4; i++) {
                pack_frame(&sample[i * 8], deg[i], now, flags);
            }
            rb_push(sample);
        }

        /* Wait for the next batch period.  The target interval may be
         * shorter than one FreeRTOS tick (10 ms at 100 Hz tick rate),
         * so we use esp_timer for sub-tick precision:
         *   1. vTaskDelay handles the coarse wait (saves CPU)
         *   2. busy-wait covers the final sub-tick remainder          */
        int64_t batch_period_us = 1000000 / freq_hz;
        int64_t elapsed_us     = esp_timer_get_time() - batch_start_us;
        int64_t remaining_us   = batch_period_us - elapsed_us;

        if (remaining_us > 2000) {
            /* Coarse sleep — subtract 1 ms margin for busy-wait tail */
            TickType_t ticks = pdMS_TO_TICKS((remaining_us - 1000) / 1000);
            if (ticks > 0) vTaskDelay(ticks);
        }
        /* Fine-grained busy-wait for the remainder */
        while ((esp_timer_get_time() - batch_start_us) < batch_period_us) {
            /* yield to equal-priority tasks while spinning */
            taskYIELD();
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
        s_client_fd = httpd_req_to_sockfd(req);
        rb_reset();
        s_acq_run = true;   /* auto-start, mirrors on_dc_open for WebRTC */
        ESP_LOGI(TAG, "WS client connected fd=%d — acquisition started", s_client_fd);
        return ESP_OK;
    }
    /* Drain any data frame the client sends — firmware does not parse them. */
    httpd_ws_frame_t pkt = {0};
    pkt.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) {
        s_acq_run   = false;
        s_client_fd = -1;
        rb_free();
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
    s_server = server;
    rb_init();   /* mutex only — buffer stays on heap until a client connects */
    httpd_uri_t uri = {
        .uri                      = "/ws",
        .method                   = HTTP_GET,
        .handler                  = ws_handler,
        .is_websocket             = true,
        .handle_ws_control_frames = false,
    };
    httpd_register_uri_handler(server, &uri);
    /* sensor_acq_task at same priority as peer_main_task so acquisitions
     * are not starved; ws_flush_task at low priority (it blocks on network). */
    xTaskCreate(sensor_acq_task, "hfhl_acq",   3072, NULL, 6, NULL);
    xTaskCreate(ws_flush_task,   "hfhl_flush",  4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "HFHL WebSocket ready at ws://192.168.4.1/ws");
}

void hfhl_ws_set_batch_params(int fpp, int freq_hz) {
    if (fpp >= 1 && fpp <= 46)       s_acq_fpp     = fpp;
    if (freq_hz >= 1 && freq_hz <= 1000) s_acq_freq_hz = freq_hz;
    ESP_LOGI(TAG, "HFHL batch params: fpp=%d freq=%dHz -> total=%dHz",
             s_acq_fpp, s_acq_freq_hz, s_acq_fpp * s_acq_freq_hz);
}

void hfhl_ws_stop(void) {
    s_acq_run   = false;
    s_client_fd = -1;
    rb_free();
    ESP_LOGI(TAG, "HFHL acquisition stopped");
}

bool hfhl_ws_is_connected(void) {
    return s_client_fd >= 0;
}
