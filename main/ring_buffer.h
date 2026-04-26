#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RB_SAMPLE_SIZE  32     /* 4 sensors × 8 bytes per timestep */
#define RB_CAPACITY      2048  /* ~64 KB DRAM; burst absorber, not persistent store */

/* Call once at startup — creates the mutex only, does NOT allocate the buffer. */
void rb_init(void);

/* Allocate the ~80 KB buffer. Call when a WebSocket client connects. No-op if
 * already allocated. Returns true on success, false on OOM. */
bool rb_alloc(void);

/* Free the buffer. Call when the WebSocket client disconnects so WebRTC's
 * mbedTLS context can reclaim the heap. No-op if not allocated. */
void rb_free(void);

void rb_push(const uint8_t *sample);          /* drop-oldest if full */
int  rb_pop_batch(uint8_t *dst, int max);     /* returns count popped */
int  rb_count(void);
void rb_reset(void);
