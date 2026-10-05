/**
 * @file atlas_imu_stream.h
 * @brief LSM6DSV16B FIFO drain over SPI3 DMA, feeding timestamped IMU samples.
 *
 * Major functions:
 * - AtlasImuStream_Init(): binds the FIFO-mode LSM6DSV16B; uses CubeMX-linked SPI3 DMA.
 * - AtlasImuStream_Service(): one non-blocking step per I/O-task cycle.
 * - AtlasImuStream_GetStats(): transport counters for health reporting.
 *
 * Each step either starts one DMA burst (status and clock-pair reads are short polled
 * transfers) or collects a finished burst, decodes it, and pushes the samples. The I/O
 * task never waits for burst data. SPI3 is dedicated to the LSM6DSV16B, so nothing
 * else can contend for the bus while a burst is in flight. If DMA cannot be set up
 * or repeatedly fails, the same bursts are read with polled SPI instead.
 *
 * Only the I/O task may call these functions; the ISR side only sets flags.
 */

#ifndef ATLAS_IMU_STREAM_H
#define ATLAS_IMU_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "atlas_lsm6dsv16b.h"
#include "atlas_sample_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Largest single burst; the FIFO keeps any remainder for the next cycle. */
#define ATLAS_IMU_STREAM_MAX_BURST_WORDS (64U)

/** @brief Transport counters. */
typedef struct
{
    bool dma_active;          /**< false when running on the polled fallback. */
    uint32_t dma_bursts;
    uint32_t polled_bursts;
    uint32_t dma_errors;
    uint32_t dma_timeouts;
    uint32_t words_read;
    uint32_t samples_produced;
    uint32_t max_fifo_words;  /**< Largest FIFO level seen: backlog indicator. */
} AtlasImuStreamStats;

/**
 * @brief Bind the sensor and select DMA if HAL_SPI_MspInit() linked SPI3 streams.
 * @param sensor LSM6DSV16B initialized with fifo_enabled.
 * @return ATLAS_OK (DMA or polled fallback ready) or ATLAS_ERROR_STATE/NULL.
 * @note Call once before the scheduler, after the sensor probe (AtlasRtos_Start()
 *       does this). Missing DMA links select the polled fallback; not an error.
 */
AtlasStatus AtlasImuStream_Init(AtlasLsm6dsv16b *sensor);

/**
 * @brief Advance the FIFO drain by one step without waiting for burst data.
 * @param ring Destination ring, or NULL to decode without publishing.
 * @param produced Destination count of samples decoded this step (may be NULL).
 * @return ATLAS_OK, ATLAS_ERROR_NOT_READY before Init, or a transport error.
 */
AtlasStatus AtlasImuStream_Service(AtlasSampleRing *ring, size_t *produced);

/**
 * @brief Copy the transport counters.
 * @param stats Destination.
 */
void AtlasImuStream_GetStats(AtlasImuStreamStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* ATLAS_IMU_STREAM_H */
