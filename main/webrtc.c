#include "webrtc.h"
#include "peer.h"
#include "peer_connection.h"
#include "app_config.h"
#if !ENABLE_WEBRTC_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
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

/**
 * Mock sine-wave parameters matching JS app's ACTIVE_JOINTS order:
 * [0] left-elbow  [1] right-elbow  [2] left-knee  [3] right-knee
 */
static const float MOCK_CENTER[]    = { 80.0f,  75.0f,  90.0f,  85.0f };
static const float MOCK_AMP[]       = { 40.0f,  38.0f,  45.0f,  42.0f };
static const float MOCK_PERIOD_MS[] = { 8000.0f, 7500.0f, 10000.0f, 9500.0f };
static const float MOCK_PHASE[]     = { 0.0f, 1.2f, 0.5f, 1.8f };

/**
 * @brief Generates mock kinematic joint angles using a phase-shifted sine wave.
 *
 * Since physical encoders are not connected in this firmware iteration, we simulate 
 * joint angles to test the high-frequency WebRTC telemetry link and the browser's 3D visualizer.
 * Each "joint" runs on a slightly different period and phase to create organic-looking motion.
 *
 * @param s Sensor index (0=left-elbow, 1=right-elbow, 2=left-knee, 3=right-knee).
 * @param t_ms Current system uptime in milliseconds, used as the time domain 'x' input.
 * @return float Mock angle strictly clamped between 0.0 and 180.0 degrees to match physical servo limits.
 */
static float mock_degrees(int s, int64_t t_ms) {
    float deg = MOCK_CENTER[s] + MOCK_AMP[s] *
                sinf(2.0f * (float)M_PI * (float)t_ms / MOCK_PERIOD_MS[s] + MOCK_PHASE[s]);
    // Clamp to the range expected by the browser visualizer.
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return deg;
}

/**
 * @brief Packs a single kinematic sensor reading into a highly optimized 8-byte payload.
 *
 * To maximize telemetry throughput over the WebRTC Data Channel, we avoid heavy JSON payloads.
 * Instead, we use a custom 64-bit binary struct, packed tightly:
 *
 * [Bits 63:50] - 14-bit Angle (0-360 degrees mapped linearly to 0-16383).
 * [Bits 49:18] - 32-bit Timestamp (microseconds since boot, allows for precise browser-side jitter buffering).
 * [Bits 17:10] - 8-bit State of Charge (SoC % mapped to 0-255).
 * [Bits 9:0]   - 10-bit Flags (Reserved for future fault codes or calibration markers).
 *
 * The final 64-bit block is serialized in strict Network Byte Order (Big-Endian) so the browser's 
 * JavaScript DataView can parse it natively without endian-swapping overhead.
 *
 * @param[out] buf Pointer to an 8-byte buffer where the serialized data will be written.
 * @param s The joint index being sampled.
 * @param t_us The exact hardware microsecond timestamp of the sample.
 */
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

/**
 * @brief Creates a default PeerConfiguration.
 *
 * @return PeerConfiguration WebRTC configuration struct.
 */
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

/**
 * @brief Callback for ICE connection state changes.
 *
 * @param state New ICE connection state.
 * @param userdata User data pointer.
 */
static void on_icestate(PeerConnectionState state, void *userdata) {
    ESP_LOGI(TAG, "ICE state: %s", peer_connection_state_to_string(state));
    // Any terminal/failed ICE state should stop telemetry transmission.
    if (state == PEER_CONNECTION_FAILED ||
        state == PEER_CONNECTION_DISCONNECTED ||
        state == PEER_CONNECTION_CLOSED) {
        g_dc_open = false;
    }
}

/**
 * @brief Callback triggered when the local description is ready.
 *
 * Called from `peer_connection_loop()` once the local answer SDP + host ICE candidates are ready.
 *
 * @param sdp The generated local SDP string.
 * @param userdata User data pointer.
 */
static void on_local_description(char *sdp, void *userdata) {
    // Replace stale cached SDP, then wake waiter in webrtc_handle_offer().
    free(g_answer_sdp);
    g_answer_sdp = sdp ? strdup(sdp) : NULL;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "local description ready (answer len=%d)", sdp ? (int)strlen(sdp) : 0);
#endif
    xSemaphoreGive(g_answer_ready);
}

/**
 * @brief Callback triggered when the data channel opens.
 *
 * @param userdata User data pointer.
 */
static void on_dc_open(void *userdata) {
  ESP_LOGI(TAG, "data channel open");
  // Reset counters when a new browser session connects.
  g_dc_open       = true;
  g_stream_active = true;
    g_tx_packets    = 0;
    g_tx_bytes      = 0;
    g_rx_messages   = 0;
}

/**
 * @brief Callback triggered when the data channel closes.
 *
 * @param userdata User data pointer.
 */
static void on_dc_close(void *userdata) {
    ESP_LOGI(TAG, "data channel closed");
    g_dc_open = false;
}

/**
 * @brief Callback triggered when a message is received on the data channel.
 *
 * @param msg The message payload.
 * @param len Length of the message payload.
 * @param userdata User data pointer.
 * @param sid Stream ID of the data channel.
 */
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

/**
 * @brief FreeRTOS task that periodically drives the libpeer state machine.
 *
 * @param pv Task parameter (unused).
 */
static void peer_main_task(void *pv) {
  while (1) {
    // libpeer requires regular polling to drive ICE and callback dispatch.
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    if (g_pc) peer_connection_loop(g_pc);
        xSemaphoreGive(g_pc_mutex);
        vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
    }
}

/**
 * @brief FreeRTOS task that streams telemetry data over the WebRTC data channel.
 *
 * @param pv Task parameter (unused).
 */
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

/**
 * @brief Registers WebRTC callbacks with the active PeerConnection.
 */
static void setup_callbacks(void) {
    // Register all callbacks immediately after creating a new PeerConnection.
    peer_connection_oniceconnectionstatechange(g_pc, on_icestate);
    peer_connection_onicecandidate(g_pc, on_local_description);
    peer_connection_ondatachannel(g_pc, on_dc_message, on_dc_open, on_dc_close);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief Initializes the WebRTC subsystem.
 *
 * @return ESP_OK on success.
 */
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

/**
 * @brief Core WebRTC signaling state machine: Processes a browser's SDP Offer and generates an Answer.
 *
 * This is the heaviest function in the system. When a browser calls `/api/offer`, this routine must:
 * 1. Acquire `g_offer_lock` to ensure we never process two concurrent signaling requests 
 *    (e.g., if the user mashes the connect button).
 * 2. Tear down any existing PeerConnection. This cleanly handles "browser refresh" scenarios 
 *    where the previous connection died silently.
 * 3. Initialize a fresh `libpeer` PeerConnection and inject the browser's remote SDP.
 * 4. Block on `g_answer_ready` while the background `peer_main_task` parses the remote SDP, 
 *    gathers local ICE host candidates (from our SoftAP IP), and formulates the local Answer.
 * 5. Extract the generated Answer SDP and hand it back to the HTTP handler to return to the browser.
 *
 * @param offer_sdp The raw SDP string received from the browser's HTTP POST body.
 * @param[out] answer_out Pointer that will be updated to point to a dynamically allocated SDP answer string.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if ICE gathering stalls, or ESP_FAIL on internal errors.
 */
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

/**
 * @brief Frees the generated answer SDP string.
 *
 * @param answer The SDP answer string to free.
 */
void webrtc_free_answer(char *answer) {
  // Ownership is transferred to caller by webrtc_handle_offer().
  free(answer);
}

/**
 * @brief Adds a trickled ICE candidate to the WebRTC connection.
 *
 * @param candidate The ICE candidate string.
 * @return ESP_OK on success, ESP_FAIL or ESP_ERR_INVALID_STATE on error.
 */
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

/**
 * @brief Checks if the WebRTC data channel is currently open.
 *
 * @return true if connected, false otherwise.
 */
bool webrtc_is_connected(void) {
    return g_dc_open;
}

/**
 * @brief Sets the State of Charge (SoC) value for telemetry transmission.
 *
 * @param soc_pct State of Charge percentage.
 */
void webrtc_set_soc(float soc_pct) {
  // Stored as shared telemetry state and packed into outgoing sensor frames.
  g_soc_pct = soc_pct;
}

/**
 * @brief Sets the streaming rate for the WebRTC data channel.
 *
 * @param rate_hz Streaming rate in Hertz (clamped between 10 and 100).
 */
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

/**
 * @brief Pauses periodic data-channel telemetry streaming.
 */
void webrtc_stop_stream(void) {
    // Pause periodic data-channel sends without tearing down WebRTC transport.
    g_stream_active = false;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "stream stopped");
#endif
}
