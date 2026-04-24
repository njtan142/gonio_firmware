#include "webrtc.h"
#include "peer.h"
#include "peer_connection.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "webrtc";
#define ENABLE_WEBRTC_TRACE 1

// Shared PeerConnection + signaling state across HTTP handlers and RTOS tasks.
static PeerConnection    *g_pc            = NULL;
static SemaphoreHandle_t  g_pc_mutex      = NULL;
static SemaphoreHandle_t  g_offer_lock    = NULL;
static SemaphoreHandle_t  g_answer_ready  = NULL;
// Temporary ownership handoff: callback writes this, API returns it to caller.
static char              *g_answer_sdp    = NULL;
// Data-channel liveness flag used by both /api/status and streaming loop.
static volatile bool      g_dc_open       = false;
// Last SoC value pushed from ui_task; encoded into outgoing telemetry frames.
static volatile float     g_soc_pct       = 100.0f;
static volatile int       g_stream_rate   = 50;   // Hz
// Send-loop gate toggled by control API and channel lifecycle callbacks.
static volatile bool      g_stream_active = true;
// Basic diagnostics counters for periodic trace logs.
static uint32_t           g_tx_packets    = 0;
static uint32_t           g_tx_bytes      = 0;
static uint32_t           g_rx_messages   = 0;

/* Mock sine-wave parameters matching JS app's ACTIVE_JOINTS order:
 * [0] left-elbow  [1] right-elbow  [2] left-knee  [3] right-knee */
static const float MOCK_CENTER[]    = { 80.0f,  75.0f,  90.0f,  85.0f };
static const float MOCK_AMP[]       = { 40.0f,  38.0f,  45.0f,  42.0f };
static const float MOCK_PERIOD_MS[] = { 8000.0f, 7500.0f, 10000.0f, 9500.0f };
static const float MOCK_PHASE[]     = { 0.0f, 1.2f, 0.5f, 1.8f };

static float mock_degrees(int s, int64_t t_ms) {
    float deg = MOCK_CENTER[s] + MOCK_AMP[s] *
                sinf(2.0f * (float)M_PI * (float)t_ms / MOCK_PERIOD_MS[s] + MOCK_PHASE[s]);
    // Clamp to the range expected by the browser visualizer.
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return deg;
}

/* Pack one 8-byte sensor frame, big-endian.
 * Layout: [63:50]=14-bit angle  [49:18]=32-bit µs ts  [17:10]=8-bit SoC  [9:0]=flags */
static void pack_frame(uint8_t *buf, int s, int64_t t_us) {
    float    deg   = mock_degrees(s, t_us / 1000);
    // 14-bit angle field maps 0..360 deg onto 0..16383 raw units.
    uint16_t raw   = (uint16_t)(deg * (16384.0f / 360.0f)) & 0x3FFF;
    // Only lower 32 bits are transmitted; receiver treats timestamp as wrapping.
    uint32_t ts    = (uint32_t)(t_us & 0xFFFFFFFFULL);
    uint8_t  soc   = (uint8_t)(g_soc_pct * 2.55f);
    uint64_t frame = ((uint64_t)raw << 50)
                   | ((uint64_t)ts  << 18)
                   | ((uint64_t)soc << 10);
    // Serialize as network-order bytes to keep browser-side unpacking simple.
    for (int i = 7; i >= 0; i--) {
        buf[i] = (uint8_t)(frame & 0xFF);
        frame >>= 8;
    }
}

static PeerConfiguration make_config(void) {
    PeerConfiguration cfg = {
        /* No STUN/TURN — local AP, host candidates are sufficient */
        .ice_servers  = {},
        .audio_codec  = CODEC_NONE,
        .video_codec  = CODEC_NONE,
        .datachannel  = DATA_CHANNEL_BINARY,
    };
    return cfg;
}

/* ── libpeer callbacks ──────────────────────────────────────────────────── */

static void on_icestate(PeerConnectionState state, void *userdata) {
    ESP_LOGI(TAG, "ICE state: %s", peer_connection_state_to_string(state));
    // Any terminal/failed ICE state should stop telemetry transmission.
    if (state == PEER_CONNECTION_FAILED ||
        state == PEER_CONNECTION_DISCONNECTED ||
        state == PEER_CONNECTION_CLOSED) {
        g_dc_open = false;
    }
}

/* Called from peer_connection_loop() (within peer_main_task) once the local
   description (answer SDP + host ICE candidates) is ready. */
static void on_local_description(char *sdp, void *userdata) {
    // Replace stale cached SDP, then wake waiter in webrtc_handle_offer().
    free(g_answer_sdp);
    g_answer_sdp = sdp ? strdup(sdp) : NULL;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "local description ready (answer len=%d)", sdp ? (int)strlen(sdp) : 0);
#endif
    xSemaphoreGive(g_answer_ready);
}

static void on_dc_open(void *userdata) {
  ESP_LOGI(TAG, "data channel open");
  // Reset counters when a new browser session connects.
  g_dc_open       = true;
  g_stream_active = true;
    g_tx_packets    = 0;
    g_tx_bytes      = 0;
    g_rx_messages   = 0;
}

static void on_dc_close(void *userdata) {
    ESP_LOGI(TAG, "data channel closed");
    g_dc_open = false;
}

static void on_dc_message(char *msg, size_t len, void *userdata, uint16_t sid) {
    g_rx_messages++;
    // Current firmware does not parse inbound payloads; we only count/log them.
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "dc rx sid=%u len=%d count=%lu", sid, (int)len, (unsigned long)g_rx_messages);
#else
    ESP_LOGD(TAG, "rx %d bytes from browser", (int)len);
#endif
}

/* ── FreeRTOS tasks ─────────────────────────────────────────────────────── */

static void peer_main_task(void *pv) {
  while (1) {
    // libpeer requires regular polling to drive ICE and callback dispatch.
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    if (g_pc) peer_connection_loop(g_pc);
        xSemaphoreGive(g_pc_mutex);
        vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
    }
}

static void streaming_task(void *pv) {
    uint8_t payload[32];
    int64_t last_stat_log_us = 0;
    while (1) {
    // Back off when channel is down or user paused stream.
    if (!g_dc_open || !g_stream_active) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    int64_t now = esp_timer_get_time();
    // Packet payload is fixed at 4 sensors x 8 bytes each.
    for (int s = 0; s < 4; s++) pack_frame(&payload[s * 8], s, now);

        xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
        if (g_pc && g_dc_open) {
        // Send exactly one sample bundle (4 sensors) each loop iteration.
        int rc = peer_connection_datachannel_send(g_pc, (char *)payload, 32);
        if (rc < 0) {
          ESP_LOGW(TAG, "dc tx failed rc=%d", rc);
        } else {
                g_tx_packets++;
                g_tx_bytes += 32;
            }
        }
        xSemaphoreGive(g_pc_mutex);

#if ENABLE_WEBRTC_TRACE
        if (last_stat_log_us == 0 || (now - last_stat_log_us) >= 2000000) {
            ESP_LOGI(TAG, "dc tx packets=%lu bytes=%lu rate=%dHz active=%d",
                     (unsigned long)g_tx_packets, (unsigned long)g_tx_bytes,
                     g_stream_rate, g_stream_active ? 1 : 0);
            last_stat_log_us = now;
        }
#endif

    // g_stream_rate is clamped by webrtc_set_stream_rate().
    vTaskDelay(pdMS_TO_TICKS(1000 / g_stream_rate));
  }
}

/* ── helpers ────────────────────────────────────────────────────────────── */

static void setup_callbacks(void) {
    // Register all callbacks immediately after creating a new PeerConnection.
    peer_connection_oniceconnectionstatechange(g_pc, on_icestate);
    peer_connection_onicecandidate(g_pc, on_local_description);
    peer_connection_ondatachannel(g_pc, on_dc_message, on_dc_open, on_dc_close);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t webrtc_init(void) {
  peer_init();

  // One mutex for PeerConnection state, one lock to serialize offer exchanges.
  g_pc_mutex     = xSemaphoreCreateMutex();
  g_offer_lock   = xSemaphoreCreateMutex();
  g_answer_ready = xSemaphoreCreateBinary();

  // peer_main_task drives libpeer state machine; streaming_task sends telemetry.
    xTaskCreate(peer_main_task, "peer_main",   8192, NULL, 6, NULL);
    xTaskCreate(streaming_task, "peer_stream", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "WebRTC initialized (libpeer, data channel only)");
    return ESP_OK;
}

esp_err_t webrtc_handle_offer(const char *offer_sdp, char **answer_out) {
  // Prevent concurrent renegotiations from overlapping shared global state.
  if (xSemaphoreTake(g_offer_lock, pdMS_TO_TICKS(100)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "handling offer (len=%d)", offer_sdp ? (int)strlen(offer_sdp) : 0);
#endif

    /* Drain any stale answer signal from a previous exchange */
  // Clear stale completion signal from any previous failed/aborted exchange.
  xSemaphoreTake(g_answer_ready, 0);
  free(g_answer_sdp);
  g_answer_sdp = NULL;

    /* Create/recreate peer connection for each new offer (handles browser refresh) */
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    g_dc_open = false;
    if (g_pc) {
        peer_connection_close(g_pc);
        peer_connection_destroy(g_pc);
        g_pc = NULL;
    }

    PeerConfiguration cfg = make_config();
    g_pc = peer_connection_create(&cfg);
    if (!g_pc) {
        xSemaphoreGive(g_pc_mutex);
        xSemaphoreGive(g_offer_lock);
    return ESP_FAIL;
  }
    setup_callbacks();
    /* Setting remote description triggers answer generation; ICE gathering
       is driven by peer_main_task calling peer_connection_loop(). */
    peer_connection_set_remote_description(g_pc, offer_sdp);
    xSemaphoreGive(g_pc_mutex);

    /* Wait for on_local_description to fire (host candidates only — fast) */
    if (xSemaphoreTake(g_answer_ready, pdMS_TO_TICKS(8000)) != pdTRUE) {
        ESP_LOGE(TAG, "timed out waiting for local description");
        xSemaphoreGive(g_offer_lock);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(g_offer_lock);

  if (!g_answer_sdp) return ESP_FAIL;
  *answer_out = g_answer_sdp;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "offer handled, answer ready (len=%d)", (int)strlen(g_answer_sdp));
#endif
    g_answer_sdp = NULL;
    return ESP_OK;
}

void webrtc_free_answer(char *answer) {
  // Ownership is transferred to caller by webrtc_handle_offer().
  free(answer);
}

esp_err_t webrtc_add_ice_candidate(const char *candidate) {
    if (!g_pc) return ESP_ERR_INVALID_STATE;
    // Add candidate under lock because g_pc can be replaced during renegotiation.
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    int rc = peer_connection_add_ice_candidate(g_pc, (char *)candidate);
    xSemaphoreGive(g_pc_mutex);
    if (rc < 0) {
        ESP_LOGW(TAG, "add ICE candidate failed rc=%d", rc);
        return ESP_FAIL;
    }
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "ICE candidate added");
#endif
    return ESP_OK;
}

bool webrtc_is_connected(void) {
    return g_dc_open;
}

void webrtc_set_soc(float soc_pct) {
  // Stored as shared telemetry state and packed into outgoing sensor frames.
  g_soc_pct = soc_pct;
}

void webrtc_set_stream_rate(int rate_hz) {
  // Keep rate bounded to protect bandwidth and task scheduling jitter.
  // Rate update is independent from stream_active (set by separate controls).
  if (rate_hz >= 10 && rate_hz <= 100) {
        g_stream_rate = rate_hz;
#if ENABLE_WEBRTC_TRACE
        ESP_LOGI(TAG, "stream rate set to %dHz", g_stream_rate);
#endif
    }
}

void webrtc_stop_stream(void) {
    // Pause periodic data-channel sends without tearing down WebRTC transport.
    g_stream_active = false;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "stream stopped");
#endif
}
