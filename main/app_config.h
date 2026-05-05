#pragma once

#define ENABLE_DISPLAY                                                         \
  1 // set to 0 to power off the display and measure baseline draw
#define ENABLE_EXT_FLASH                                                       \
  0 // set to 0 to bypass the external flash and web server completely
#define ENABLE_COULOMB_LOG                                                     \
  0 // set to 0 to disable periodic coulomb console logs

#define ENABLE_WEBSERVER_LOG                                                   \
  1 // set to 0 to disable web server logs to save memory
#define ENABLE_WEBRTC_LOG 1 // set to 0 to disable WebRTC logs to save memory
#define ENABLE_SYSTEM_LOG                                                      \
  1 // set to 0 to disable core system logs to save memory
#define ENABLE_PC_DATA_TRACE                                                   \
  0 // set to 1 to enable low-level WebRTC data packet tracing
#define ENABLE_PC_BIO_TRACE                                                    \
  0 // set to 1 to enable high-frequency BIO send/recv logs
#define ENABLE_WS_DATA_TRACE                                                   \
  0 // set to 1 to log decoded angles for every WS flush batch
#define BATTERY_CAPACITY_MAH 450.0 // Nominal battery capacity in mAh
#define BATTERY_CAPACITY_MAS                                                   \
  (BATTERY_CAPACITY_MAH * 3600.0) // Capacity in mAs (milliampere-seconds)
