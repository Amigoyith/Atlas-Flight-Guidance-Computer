/**
 * @file atlas_sample_ring.c
 * @brief Single-producer/single-consumer record ring with explicit publication fences.
 *
 * Major functions:
 * - AtlasSampleRing_Push(): copies a record, then publishes the new head.
 * - AtlasSampleRing_Pop(): copies records, then releases their slots via tail.
 */

#include "atlas_sample_ring.h"

#include <stdatomic.h>
#include <string.h>

/**
 * @brief Bind storage and reset counters.
 * @param ring Ring to initialize.
 * @param storage Caller-owned record storage.
 * @param element_size Record size in bytes.
 * @param capacity Power-of-two record count.
 * @return true on success.
 */
bool AtlasSampleRing_Init(AtlasSampleRing *ring,
                          void *storage,
                          size_t element_size,
                          uint32_t capacity)
{
    if ((ring == NULL) || (storage == NULL) || (element_size == 0U) ||
        (capacity < 2U) || (capacity > (1UL << 30)) ||
        ((capacity & (capacity - 1U)) != 0U))
    {
        return false;
    }
    ring->storage = (uint8_t *)storage;
    ring->element_size = element_size;
    ring->capacity = capacity;
    ring->head = 0U;
    ring->tail = 0U;
    ring->overflows = 0U;
    return true;
}

/**
 * @brief Append one record; the producer owns head and never touches tail.
 * @param ring Initialized ring.
 * @param element Record to copy.
 * @return true when stored.
 */
bool AtlasSampleRing_Push(AtlasSampleRing *ring, const void *element)
{
    uint32_t head;

    if ((ring == NULL) || (ring->storage == NULL) || (element == NULL))
    {
        return false;
    }
    head = ring->head;
    if ((uint32_t)(head - ring->tail) >= ring->capacity)
    {
        ++ring->overflows;
        return false;
    }
    memcpy(&ring->storage[(size_t)(head & (ring->capacity - 1U)) * ring->element_size],
           element, ring->element_size);
    /* The record must be complete before the consumer can observe the new head. */
    atomic_thread_fence(memory_order_release);
    ring->head = head + 1U;
    return true;
}

/**
 * @brief Remove records in order; the consumer owns tail and never touches head.
 * @param ring Initialized ring.
 * @param destination Buffer for max_count records.
 * @param max_count Maximum records to copy.
 * @return Number of records copied.
 */
size_t AtlasSampleRing_Pop(AtlasSampleRing *ring, void *destination, size_t max_count)
{
    uint8_t *out = (uint8_t *)destination;
    uint32_t tail;
    uint32_t available;
    size_t copied = 0U;

    if ((ring == NULL) || (ring->storage == NULL) || (destination == NULL))
    {
        return 0U;
    }
    tail = ring->tail;
    available = ring->head - tail;
    /* Observe the head before reading the records it publishes. */
    atomic_thread_fence(memory_order_acquire);
    while ((copied < max_count) && (copied < available))
    {
        memcpy(&out[copied * ring->element_size],
               &ring->storage[(size_t)((tail + copied) & (ring->capacity - 1U)) *
                              ring->element_size],
               ring->element_size);
        ++copied;
    }
    /* Finish reading the slots before handing them back to the producer. */
    atomic_thread_fence(memory_order_release);
    ring->tail = tail + (uint32_t)copied;
    return copied;
}

/**
 * @brief Report unread records.
 * @param ring Initialized ring.
 * @return Unread record count, or 0 for NULL.
 */
uint32_t AtlasSampleRing_Count(const AtlasSampleRing *ring)
{
    return (ring == NULL) ? 0U : (uint32_t)(ring->head - ring->tail);
}
