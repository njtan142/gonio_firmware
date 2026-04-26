#pragma once
#include <stdbool.h>
#include "esp_http_server.h"

/* Register /ws URI, create sensor_acq_task and ws_flush_task.
 * Acquisition auto-starts when a client connects and auto-stops on disconnect. */
void hfhl_ws_init(httpd_handle_t server);

/* Update batching parameters without changing run state.
 * Called from /api/start when slider params change.
 * fpp = frames per batch, freq_hz = batch send frequency. */
void hfhl_ws_set_batch_params(int fpp, int freq_hz);

/* Force-stop acquisition regardless of client state.
 * Called from /api/stop. */
void hfhl_ws_stop(void);

/* True if a WebSocket client is currently connected on /ws. */
bool hfhl_ws_is_connected(void);
