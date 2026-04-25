#pragma once

#include "esp_err.h"
#include <stdbool.h>

esp_err_t webrtc_init(void);

/* Feed a remote SDP offer from the browser; blocks until local answer is ready.
 * Caller must free *answer_out with webrtc_free_answer(). */
esp_err_t webrtc_handle_offer(const char *offer_sdp, char **answer_out);
void      webrtc_free_answer(char *answer);

/* Add a browser ICE candidate received via HTTP. */
esp_err_t webrtc_add_ice_candidate(const char *candidate);

/* State queries for /api/status. */
bool webrtc_is_connected(void);

/* Called from ui_task to encode real SoC into sensor packets. */
void webrtc_set_soc(float soc_pct);

/* Called from /api/start and /api/stop handlers. */
void webrtc_set_batch_params(int frames_per_packet, int packet_freq_hz);
void webrtc_stop_stream(void);
