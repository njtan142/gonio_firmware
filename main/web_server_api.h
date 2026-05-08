#pragma once

#include "esp_http_server.h"

extern float g_zero_offsets[4];
extern float g_scale_factors[4];

#define MAX_LUT_POINTS 16

typedef struct {
    float raw;
    float physical;
} lut_point_t;

typedef struct {
    uint8_t num_points;
    lut_point_t points[MAX_LUT_POINTS];
} sensor_lut_t;

extern sensor_lut_t g_luts[4];

void web_server_api_init(void);
void web_server_register_api_routes(httpd_handle_t server);
