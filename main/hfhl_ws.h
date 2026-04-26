#pragma once
#include <stdbool.h>
#include "esp_http_server.h"

/* Register /ws URI, create sensor_acq_task and ws_flush_task.
 * Acquisition auto-starts when a client connects and auto-stops on disconnect. */
void hfhl_ws_init(httpd_handle_t server);

/* Update the acquisition rate without changing run state.
 * Called from /api/start when slider params change. */
void hfhl_ws_set_rate(int acq_hz);

/* Force-stop acquisition regardless of client state.
 * Called from /api/stop. */
void hfhl_ws_stop(void);

/* True if a WebSocket client is currently connected on /ws. */
bool hfhl_ws_is_connected(void);
