#include "webrtc.h"
#include "mt6701.h"
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
static volatile float     g_soc_pct            = 100.0f;
// Batching parameters: fpp timesteps are packed into one DataChannel send.
static volatile int       g_frames_per_packet  = 1;    // timesteps per send (1–46)
static volatile int       g_packet_freq_hz     = 60;   // packet send rate (Hz, 10–400)
// Send-loop gate toggled by control API and channel lifecycle callbacks.
static volatile bool      g_stream_active = true;
// Basic diagnostics counters for periodic trace logs.
static uint32_t           g_tx_packets    = 0;
static uint32_t           g_tx_bytes      = 0;
static uint32_t           g_rx_messages   = 0;

/**
 * @brief Packs one sensor reading into the 8-byte binary telemetry frame.
 *
 * Frame layout (big-endian, 64 bits):
 *   [63:50] 14-bit angle  (0–16383 → 0–360°)
 *   [49:18] 32-bit timestamp (µs, lower 32 bits — wraps ~71 min)
 *   [17:10]  8-bit SoC    (0–255 → 0–100%)
 *   [ 9: 6]  sensor 0–3 magnetic error flags  (bit 9 = sensor 0, …, bit 6 = sensor 3)
 *   [ 5: 0]  reserved
 *
 * @param buf    8-byte output buffer.
 * @param deg    Angle in degrees for this sensor slot.
 * @param t_us   Hardware timestamp (esp_timer_get_time()).
 * @param flags  10-bit status flags shared across the 4-sensor bundle.
 */
static void pack_frame(uint8_t *buf, float deg, int64_t t_us, uint16_t flags) {
    uint16_t raw   = (uint16_t)(deg * (16384.0f / 360.0f)) & 0x3FFF;
    uint32_t ts    = (uint32_t)(t_us & 0xFFFFFFFFULL);
    uint8_t  soc   = (uint8_t)(g_soc_pct * 2.55f);
    uint64_t frame = ((uint64_t)raw          << 50)
                   | ((uint64_t)ts           << 18)
                   | ((uint64_t)soc          << 10)
                   | ((uint64_t)(flags & 0x3FF));
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

// TODO (non-critical): Firefox compatibility.
// The DTLS/SCTP handshake works reliably on Chrome but fails or stalls on
// Firefox. Likely differences in DTLS flight timing, SCTP parameter
// negotiation, or ICE candidate handling. Needs packet-level investigation
// with Firefox's about:webrtc internals.

// TODO (non-critical): Graceful browser close/refresh handling.
// When the browser tab is closed or refreshed, the peer disappears without
// sending a DTLS close_notify or SCTP ABORT. The streaming_task continues
// to call peer_connection_datachannel_send(), which calls dtls_srtp_write(),
// which calls agent_send() on a now-dead UDP path — flooding the log with
// "Failed to sendto" errors. Fix options:
//   1. Detect N consecutive send failures and auto-teardown the PeerConnection.
//   2. Use CONFIG_KEEPALIVE_TIMEOUT (STUN binding request timeout) to detect
//      the dead peer and transition to PEER_CONNECTION_CLOSED.
//   3. Add a /api/disconnect endpoint for the browser's beforeunload event.

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
    // NOTE: browser close/refresh does NOT trigger these states — see TODO above.
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
    // Static batch buffer: max 46 timesteps × 32 bytes = 1472 bytes (MTU).
    // Static keeps it off the task stack, which is limited to 3 KB.
    static uint8_t batch_buf[46 * 32];
    int64_t last_stat_log_us = 0;

    while (1) {
        if (!g_dc_open || !g_stream_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Snapshot both params atomically so the batch is self-consistent.
        int fpp       = g_frames_per_packet;
        int freq_hz   = g_packet_freq_hz;
        int batch_len = fpp * 32;

        // Acquire fpp timesteps back-to-back; each gets its own hardware
        // timestamp so the receiver can recover the real acquisition time.
        for (int f = 0; f < fpp; f++) {
            int64_t  now   = esp_timer_get_time();
            float    deg[4];
            uint16_t flags = 0;
            for (int s = 0; s < 4; s++) {
                deg[s] = mt6701_get_degrees(s);
                if (mt6701_has_error(s)) flags |= (uint16_t)(1u << (9 - s));
            }
            for (int s = 0; s < 4; s++) {
                pack_frame(&batch_buf[f * 32 + s * 8], deg[s], now, flags);
            }
        }

        xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
        if (g_pc && g_dc_open) {
            int rc = peer_connection_datachannel_send(g_pc, (char *)batch_buf, batch_len);
            if (rc < 0) {
                ESP_LOGW(TAG, "dc tx failed rc=%d", rc);
            } else {
                g_tx_packets++;
                g_tx_bytes += batch_len;
            }
        }
        xSemaphoreGive(g_pc_mutex);

#if ENABLE_WEBRTC_TRACE
        int64_t now_log = esp_timer_get_time();
        if (last_stat_log_us == 0 || (now_log - last_stat_log_us) >= 2000000) {
            ESP_LOGI(TAG, "dc tx pkts=%lu bytes=%lu fpp=%d pkt_hz=%d total_hz=%d active=%d",
                     (unsigned long)g_tx_packets, (unsigned long)g_tx_bytes,
                     fpp, freq_hz, fpp * freq_hz, g_stream_active ? 1 : 0);
            last_stat_log_us = now_log;
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(1000 / freq_hz));
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
 * @brief Sets the batching parameters for the WebRTC streaming task.
 *
 * @param frames_per_packet Timesteps to batch per DataChannel send (1–46).
 *        46 is the MTU limit: 46 × 32 = 1472 bytes.
 * @param packet_freq_hz    How often to send a batch packet (10–400 Hz).
 *        Total readings/sec = frames_per_packet × packet_freq_hz.
 */
void webrtc_set_batch_params(int frames_per_packet, int packet_freq_hz) {
    if (frames_per_packet >= 1 && frames_per_packet <= 46)
        g_frames_per_packet = frames_per_packet;
    if (packet_freq_hz >= 10 && packet_freq_hz <= 400)
        g_packet_freq_hz = packet_freq_hz;
#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "batch params: fpp=%d pkt_hz=%d -> total=%dHz",
             g_frames_per_packet, g_packet_freq_hz,
             g_frames_per_packet * g_packet_freq_hz);
#endif
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

void webrtc_teardown(void) {
    g_stream_active = false;
    g_dc_open       = false;

    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    if (g_pc) {
        peer_connection_close(g_pc);
        peer_connection_destroy(g_pc);
        g_pc = NULL;
    }
    xSemaphoreGive(g_pc_mutex);

    free(g_answer_sdp);
    g_answer_sdp = NULL;

#if ENABLE_WEBRTC_TRACE
    ESP_LOGI(TAG, "PeerConnection torn down — heap reclaimed");
#endif
}
