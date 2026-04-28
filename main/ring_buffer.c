#include "ring_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

/* Heap-allocated on WS connect, freed on WS disconnect — keeps the DTLS heap
 * available during WebRTC sessions (§7.5). NULL when no WS client is active. */
static uint8_t          *s_buf   = NULL;
static int               s_head  = 0;
static int               s_tail  = 0;
static int               s_count = 0;
static int               s_drops = 0;
static SemaphoreHandle_t s_mutex = NULL;

/**
 * @brief Initializes the synchronization primitive used by the ring buffer.
 *
 * Must be called exactly once before any other ring buffer functions are used.
 * Creates the FreeRTOS mutex that guarantees thread-safe read/write operations.
 */
void rb_init(void) {
    // Only create the mutex if it hasn't been created yet to prevent memory leaks
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

/**
 * @brief Allocates the backing memory for the ring buffer.
 *
 * This function allocates heap memory for the buffer based on predefined
 * capacity (RB_CAPACITY) and sample size (RB_SAMPLE_SIZE). It is idempotent; 
 * if the buffer is already allocated, it simply resets the state.
 *
 * @return true if allocation was successful or already allocated, false otherwise.
 */
bool rb_alloc(void) {
    // If the buffer is already allocated, resetting it is sufficient (idempotent operation)
    if (s_buf) { rb_reset(); return true; }  /* idempotent */
    
    // Allocate the total memory required based on capacity and individual sample size
    s_buf = malloc((size_t)RB_CAPACITY * RB_SAMPLE_SIZE);
    
    // Return false if memory allocation failed (e.g., out of memory)
    if (!s_buf) return false;
    
    // Initialize all pointers and counters to zero for the fresh buffer
    s_head = s_tail = s_count = s_drops = 0;
    return true;
}

/**
 * @brief Frees the ring buffer memory and clears its internal state.
 *
 * Safely deallocates the buffer memory while holding the mutex to ensure 
 * no active pushes or pops are interrupted. Resets all pointers and counters.
 */
void rb_free(void) {
    // Acquire the mutex, waiting indefinitely if necessary, to ensure exclusive access
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    // Free the allocated heap memory
    free(s_buf);
    
    // Reset the buffer pointer to NULL to indicate it's unallocated
    s_buf   = NULL;
    
    // Reset internal state pointers and counters
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    s_drops = 0;
    
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
}

/**
 * @brief Pushes a single sensor sample into the ring buffer.
 *
 * Copies the provided sample into the buffer at the current head position. 
 * If the buffer is full, the sample is dropped and the drop counter is incremented.
 *
 * @param sample Pointer to the sensor sample data to push. Must be exactly RB_SAMPLE_SIZE bytes.
 */
void rb_push(const uint8_t *sample) {
    // Acquire the mutex for thread-safe modification
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    // Check if the buffer is unallocated or already full
    if (!s_buf || s_count >= RB_CAPACITY) {
        // If the buffer exists but is full, record this as a dropped sample
        if (s_buf) s_drops++;
        
        // Release the mutex and abort the push operation
        xSemaphoreGive(s_mutex);
        return;
    }
    
    // Copy the incoming sample into the buffer at the current head offset
    memcpy(s_buf + (size_t)s_head * RB_SAMPLE_SIZE, sample, RB_SAMPLE_SIZE);
    
    // Advance the head pointer, wrapping around if it reaches the capacity
    s_head = (s_head + 1) % RB_CAPACITY;
    
    // Increment the total count of unread samples
    s_count++;
    
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
}

/**
 * @brief Pops a batch of samples from the ring buffer into a destination array.
 *
 * Retrieves up to `max` contiguous samples from the tail of the buffer. If fewer 
 * than `max` samples are available, it returns all available samples.
 *
 * @param dst Pointer to the destination buffer where samples will be copied.
 * @param max The maximum number of samples to pop.
 * @return The actual number of samples successfully popped and copied into `dst`.
 */
int rb_pop_batch(uint8_t *dst, int max) {
    // Acquire the mutex for thread-safe access
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    // If the buffer hasn't been allocated, release the mutex and return 0 samples
    if (!s_buf) { xSemaphoreGive(s_mutex); return 0; }
    
    // Determine the actual number of samples to pop (minimum of requested max and available count)
    int n = (s_count < max) ? s_count : max;
    
    // Copy the available samples into the destination array
    for (int i = 0; i < n; i++) {
        // Copy one sample from the current tail position
        memcpy(dst + (size_t)i * RB_SAMPLE_SIZE,
               s_buf + (size_t)s_tail * RB_SAMPLE_SIZE,
               RB_SAMPLE_SIZE);
               
        // Advance the tail pointer, wrapping around if it reaches the capacity
        s_tail = (s_tail + 1) % RB_CAPACITY;
    }
    
    // Decrease the total count by the number of samples popped
    s_count -= n;
    
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
    
    // Return the actual number of samples popped
    return n;
}

/**
 * @brief Gets the current number of samples available in the ring buffer.
 *
 * @return The number of unread samples currently stored in the buffer.
 */
int rb_count(void) {
    // Acquire the mutex to read the count thread-safely
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int c = s_count;
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
    return c;
}

/**
 * @brief Resets the ring buffer's internal pointers and counters.
 *
 * Clears the buffer logically by resetting the head, tail, count, and drop 
 * counters to zero. The allocated memory remains intact.
 */
void rb_reset(void) {
    // Acquire the mutex to protect state modifications
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    // Reset all internal tracking variables to zero
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    s_drops = 0;
    
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
}

/**
 * @brief Retrieves the number of dropped samples due to buffer overflows.
 *
 * @return The count of dropped samples since the last reset or allocation.
 */
int rb_get_drops(void) {
    // Acquire the mutex to read the drops thread-safely
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int d = s_drops;
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
    return d;
}

/**
 * @brief Resets the dropped sample counter to zero.
 */
void rb_reset_drops(void) {
    // Acquire the mutex to protect the reset operation
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    // Clear the accumulated drop count
    s_drops = 0;
    
    // Release the mutex lock
    xSemaphoreGive(s_mutex);
}
