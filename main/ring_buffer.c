#include "ring_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

/* Static BSS allocation — sized at link time, no heap fragmentation risk.
 * 512 × 32 B = 16 KB, which coexists with the WebRTC DTLS heap (§7.5). */
static uint8_t           s_buf[RB_CAPACITY * RB_SAMPLE_SIZE];
static int               s_head  = 0;
static int               s_tail  = 0;
static int               s_count = 0;
static int               s_drops = 0;
static SemaphoreHandle_t s_mutex = NULL;

void rb_init(void) {
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

/* rb_alloc / rb_free kept for call-site compatibility — buffer is always
 * present in BSS so alloc is a no-op and free just resets state. */
bool rb_alloc(void) {
    rb_reset();
    return true;
}

void rb_free(void) {
    rb_reset();
}

void rb_push(const uint8_t *sample) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_count >= RB_CAPACITY) {
        /* drop-newest: buffer full — discard incoming sample to protect
         * data already queued and waiting to be sent (HFHL semantics). */
        s_drops++;
        xSemaphoreGive(s_mutex);
        return;
    }
    memcpy(s_buf + (size_t)s_head * RB_SAMPLE_SIZE, sample, RB_SAMPLE_SIZE);
    s_head = (s_head + 1) % RB_CAPACITY;
    s_count++;
    xSemaphoreGive(s_mutex);
}

int rb_pop_batch(uint8_t *dst, int max) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = (s_count < max) ? s_count : max;
    for (int i = 0; i < n; i++) {
        memcpy(dst + (size_t)i * RB_SAMPLE_SIZE,
               s_buf + (size_t)s_tail * RB_SAMPLE_SIZE,
               RB_SAMPLE_SIZE);
        s_tail = (s_tail + 1) % RB_CAPACITY;
    }
    s_count -= n;
    xSemaphoreGive(s_mutex);
    return n;
}

int rb_count(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int c = s_count;
    xSemaphoreGive(s_mutex);
    return c;
}

void rb_reset(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    s_drops = 0;
    xSemaphoreGive(s_mutex);
}

int rb_get_drops(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int d = s_drops;
    xSemaphoreGive(s_mutex);
    return d;
}

void rb_reset_drops(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_drops = 0;
    xSemaphoreGive(s_mutex);
}
