#include "ring_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

/* Heap pointer — NULL when no WebSocket client is active. */
static uint8_t          *s_buf  = NULL;
static int               s_head  = 0;
static int               s_tail  = 0;
static int               s_count = 0;
static SemaphoreHandle_t s_mutex = NULL;

void rb_init(void) {
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

bool rb_alloc(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_buf) {
        s_buf = malloc((size_t)RB_CAPACITY * RB_SAMPLE_SIZE);
    }
    bool ok = s_buf != NULL;
    xSemaphoreGive(s_mutex);
    return ok;
}

void rb_free(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    free(s_buf);
    s_buf   = NULL;
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    xSemaphoreGive(s_mutex);
}

void rb_push(const uint8_t *sample) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_buf) { xSemaphoreGive(s_mutex); return; }
    memcpy(s_buf + (size_t)s_head * RB_SAMPLE_SIZE, sample, RB_SAMPLE_SIZE);
    s_head = (s_head + 1) % RB_CAPACITY;
    if (s_count < RB_CAPACITY) {
        s_count++;
    } else {
        /* drop-oldest: advance tail past the overwritten slot */
        s_tail = (s_tail + 1) % RB_CAPACITY;
    }
    xSemaphoreGive(s_mutex);
}

int rb_pop_batch(uint8_t *dst, int max) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_buf) { xSemaphoreGive(s_mutex); return 0; }
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
    xSemaphoreGive(s_mutex);
}
