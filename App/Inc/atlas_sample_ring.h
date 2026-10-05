/**
 * @file atlas_sample_ring.h
 * @brief Lock-free single-producer/single-consumer ring for fixed-size records.
 *
 * Major functions:
 * - AtlasSampleRing_Init(): binds caller-owned power-of-two storage.
 * - AtlasSampleRing_Push(): producer-only append; a full ring drops the NEW record.
 * - AtlasSampleRing_Pop(): consumer-only bulk removal in arrival order.
 *
 * Exactly one task may push and exactly one other task may pop. The producer never
 * overwrites unread records, so the consumer detects loss from the records' own
 * sequence numbers and the producer's overflow counter.
 */

#ifndef ATLAS_SAMPLE_RING_H
#define ATLAS_SAMPLE_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Ring state; head is written only by the producer, tail only by the consumer. */
typedef struct
{
    uint8_t *storage;
    size_t element_size;
    uint32_t capacity;           /**< Power of two, at most 2^30. */
    volatile uint32_t head;      /**< Total records pushed (free-running). */
    volatile uint32_t tail;      /**< Total records popped (free-running). */
    volatile uint32_t overflows; /**< Records refused because the ring was full. */
} AtlasSampleRing;

/**
 * @brief Bind storage and reset counters.
 * @param ring Ring to initialize.
 * @param storage Caller-owned buffer of capacity * element_size bytes.
 * @param element_size Record size in bytes; nonzero.
 * @param capacity Record count; power of two from 2 to 2^30.
 * @return true on success; false leaves the ring unusable.
 * @note Call before either task starts using the ring.
 */
bool AtlasSampleRing_Init(AtlasSampleRing *ring,
                          void *storage,
                          size_t element_size,
                          uint32_t capacity);

/**
 * @brief Append one record (producer task only).
 * @param ring Initialized ring.
 * @param element Record to copy.
 * @return true when stored; false when full (counted in overflows) or invalid.
 */
bool AtlasSampleRing_Push(AtlasSampleRing *ring, const void *element);

/**
 * @brief Remove up to max_count records in arrival order (consumer task only).
 * @param ring Initialized ring.
 * @param destination Buffer for max_count records.
 * @param max_count Maximum records to copy.
 * @return Number of records copied.
 */
size_t AtlasSampleRing_Pop(AtlasSampleRing *ring, void *destination, size_t max_count);

/**
 * @brief Report the number of unread records; exact only from producer or consumer.
 * @param ring Initialized ring.
 * @return Unread record count.
 */
uint32_t AtlasSampleRing_Count(const AtlasSampleRing *ring);

#ifdef __cplusplus
}
#endif

#endif /* ATLAS_SAMPLE_RING_H */
