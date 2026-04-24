#pragma once

#define ENABLE_DISPLAY                                                         \
  1 // set to 0 to power off the display and measure baseline draw
#define ENABLE_COULOMB_LOG                                                     \
  0 // set to 0 to disable periodic coulomb console logs
#define BATTERY_CAPACITY_MAH 450.0
#define BATTERY_CAPACITY_MAS (BATTERY_CAPACITY_MAH * 3600.0)
