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

static PeerConnection    *g_pc            = NULL;
static SemaphoreHandle_t  g_pc_mutex      = NULL;
static SemaphoreHandle_t  g_offer_lock    = NULL;
static SemaphoreHandle_t  g_answer_ready  = NULL;
static char              *g_answer_sdp    = NULL;
static volatile bool      g_dc_open       = false;
static volatile float     g_soc_pct       = 100.0f;
static volatile int       g_stream_rate   = 50;   // Hz
static volatile bool      g_stream_active = true;

/* Mock sine-wave parameters matching JS app's ACTIVE_JOINTS order:
 * [0] left-elbow  [1] right-elbow  [2] left-knee  [3] right-knee */
static const float MOCK_CENTER[]    = { 80.0f,  75.0f,  90.0f,  85.0f };
static const float MOCK_AMP[]       = { 40.0f,  38.0f,  45.0f,  42.0f };
static const float MOCK_PERIOD_MS[] = { 8000.0f, 7500.0f, 10000.0f, 9500.0f };
static const float MOCK_PHASE[]     = { 0.0f, 1.2f, 0.5f, 1.8f };

static float mock_degrees(int s, int64_t t_ms) {
    float deg = MOCK_CENTER[s] + MOCK_AMP[s] *
                sinf(2.0f * (float)M_PI * (float)t_ms / MOCK_PERIOD_MS[s] + MOCK_PHASE[s]);
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return deg;
}

/* Pack one 8-byte sensor frame, big-endian.
 * Layout: [63:50]=14-bit angle  [49:18]=32-bit µs ts  [17:10]=8-bit SoC  [9:0]=flags */
static void pack_frame(uint8_t *buf, int s, int64_t t_us) {
    float    deg   = mock_degrees(s, t_us / 1000);
    uint16_t raw   = (uint16_t)(deg * (16384.0f / 360.0f)) & 0x3FFF;
    uint32_t ts    = (uint32_t)(t_us & 0xFFFFFFFFULL);
    uint8_t  soc   = (uint8_t)(g_soc_pct * 2.55f);
    uint64_t frame = ((uint64_t)raw << 50)
                   | ((uint64_t)ts  << 18)
                   | ((uint64_t)soc << 10);
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
    if (state == PEER_CONNECTION_FAILED ||
        state == PEER_CONNECTION_DISCONNECTED ||
        state == PEER_CONNECTION_CLOSED) {
        g_dc_open = false;
    }
}

/* Called from peer_connection_loop() (within peer_main_task) once the local
   description (answer SDP + host ICE candidates) is ready. */
static void on_local_description(char *sdp, void *userdata) {
    free(g_answer_sdp);
    g_answer_sdp = sdp ? strdup(sdp) : NULL;
    xSemaphoreGive(g_answer_ready);
}

static void on_dc_open(void *userdata) {
    ESP_LOGI(TAG, "data channel open");
    g_dc_open       = true;
    g_stream_active = true;
}

static void on_dc_close(void *userdata) {
    ESP_LOGI(TAG, "data channel closed");
    g_dc_open = false;
}

static void on_dc_message(char *msg, size_t len, void *userdata, uint16_t sid) {
    ESP_LOGD(TAG, "rx %d bytes from browser", (int)len);
}

/* ── FreeRTOS tasks ─────────────────────────────────────────────────────── */

static void peer_main_task(void *pv) {
    while (1) {
        xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
        if (g_pc) peer_connection_loop(g_pc);
        xSemaphoreGive(g_pc_mutex);
        vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
    }
}

static void streaming_task(void *pv) {
    uint8_t payload[32];
    while (1) {
        if (!g_dc_open || !g_stream_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int64_t now = esp_timer_get_time();
        for (int s = 0; s < 4; s++) pack_frame(&payload[s * 8], s, now);

        xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
        if (g_pc && g_dc_open) {
            peer_connection_datachannel_send(g_pc, (char *)payload, 32);
        }
        xSemaphoreGive(g_pc_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000 / g_stream_rate));
    }
}

/* ── helpers ────────────────────────────────────────────────────────────── */

static void setup_callbacks(void) {
    peer_connection_oniceconnectionstatechange(g_pc, on_icestate);
    peer_connection_onicecandidate(g_pc, on_local_description);
    peer_connection_ondatachannel(g_pc, on_dc_message, on_dc_open, on_dc_close);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t webrtc_init(void) {
    peer_init();

    g_pc_mutex     = xSemaphoreCreateMutex();
    g_offer_lock   = xSemaphoreCreateMutex();
    g_answer_ready = xSemaphoreCreateBinary();

    PeerConfiguration cfg = make_config();
    g_pc = peer_connection_create(&cfg);
    if (!g_pc) {
        ESP_LOGE(TAG, "peer_connection_create failed");
        return ESP_FAIL;
    }
    setup_callbacks();

    xTaskCreate(peer_main_task, "peer_main",   8192, NULL, 6, NULL);
    xTaskCreate(streaming_task, "peer_stream", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "WebRTC initialized (libpeer, data channel only)");
    return ESP_OK;
}

esp_err_t webrtc_handle_offer(const char *offer_sdp, char **answer_out) {
    if (!g_pc) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(g_offer_lock, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    /* Drain any stale answer signal from a previous exchange */
    xSemaphoreTake(g_answer_ready, 0);
    free(g_answer_sdp);
    g_answer_sdp = NULL;

    /* Recreate the peer connection on each new offer (handles browser refresh) */
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    g_dc_open = false;
    peer_connection_close(g_pc);
    peer_connection_destroy(g_pc);

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
    g_answer_sdp = NULL;
    return ESP_OK;
}

void webrtc_free_answer(char *answer) {
    free(answer);
}

esp_err_t webrtc_add_ice_candidate(const char *candidate) {
    if (!g_pc) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(g_pc_mutex, portMAX_DELAY);
    peer_connection_add_ice_candidate(g_pc, (char *)candidate);
    xSemaphoreGive(g_pc_mutex);
    return ESP_OK;
}

bool webrtc_is_connected(void) {
    return g_dc_open;
}

void webrtc_set_soc(float soc_pct) {
    g_soc_pct = soc_pct;
}

void webrtc_set_stream_rate(int rate_hz) {
    if (rate_hz >= 10 && rate_hz <= 100) g_stream_rate = rate_hz;
}

void webrtc_stop_stream(void) {
    g_stream_active = false;
}
