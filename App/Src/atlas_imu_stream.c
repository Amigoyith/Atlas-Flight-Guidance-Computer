/**
 * @file atlas_imu_stream.c
 * @brief SPI3 DMA transport for LSM6DSV16B FIFO bursts, with a polled fallback.
 *
 * Major functions:
 * - AtlasImuStream_Init(): binds the sensor and checks for CubeMX-linked SPI3 DMA.
 * - AtlasImuStream_Service(): IDLE -> (status, clock pair, start burst) -> BUSY ->
 *   (collect, decode, publish) -> IDLE.
 * - HAL_SPI_TxRxCpltCallback()/HAL_SPI_ErrorCallback(): flag-only ISR completion.
 *
 * The DMA hardware is owned by CubeMX (Atlas.ioc): DMA1 Stream2 = SPI3_RX and
 * Stream3 = SPI3_TX, byte-wide, normal mode, high priority, NVIC priority 6 for both
 * streams and SPI3. HAL_SPI_MspInit() links them to hspi3, MX_DMA_Init() enables the
 * stream IRQs and stm32h7xx_it.c dispatches DMA1_Stream2/3 and SPI3 interrupts. Those
 * handlers never call the kernel (priority 6 is below the syscall limit of 5 anyway).
 *
 * Burst buffers live in the AXI-SRAM `.atlas_dma` section because DMA1 cannot reach
 * DTCM. They are zeroed in complete 64-bit ECC words before any byte access. The
 * D-cache is disabled on Atlas, so no cache maintenance is required; enabling it
 * later requires clean/invalidate around each burst.
 */

#include "atlas_imu_stream.h"

#include <string.h>

#include "atlas_time.h"

#define ATLAS_IMU_STREAM_BURST_BYTES \
    (1U + (ATLAS_IMU_STREAM_MAX_BURST_WORDS * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES))
/* Round up to whole 32-byte lines of 64-bit ECC words. */
#define ATLAS_IMU_STREAM_BUFFER_BYTES   (((ATLAS_IMU_STREAM_BURST_BYTES + 31U) / 32U) * 32U)
#define ATLAS_IMU_STREAM_TIMEOUT_MS     (5U)
#define ATLAS_IMU_STREAM_MAX_DMA_FAILURES (3U)

/** @brief Burst buffer initialized in complete ECC words before DMA/subword access. */
typedef union
{
    uint64_t ecc[ATLAS_IMU_STREAM_BUFFER_BYTES / 8U];
    uint8_t bytes[ATLAS_IMU_STREAM_BUFFER_BYTES];
} AtlasImuStreamBuffer;

#if defined(__ICCARM__)
#pragma location = ".atlas_dma"
#pragma data_alignment = 32
static __no_init AtlasImuStreamBuffer atlas_imu_tx;
#pragma location = ".atlas_dma"
#pragma data_alignment = 32
static __no_init AtlasImuStreamBuffer atlas_imu_rx;
#elif defined(__GNUC__)
static AtlasImuStreamBuffer atlas_imu_tx __attribute__((section(".atlas_dma"), aligned(32)));
static AtlasImuStreamBuffer atlas_imu_rx __attribute__((section(".atlas_dma"), aligned(32)));
#else
#error "Define and verify Atlas DMA placement for this compiler before enabling IMU DMA."
#endif

typedef enum
{
    ATLAS_IMU_STREAM_IDLE = 0,
    ATLAS_IMU_STREAM_BUSY
} AtlasImuStreamState;

static AtlasLsm6dsv16b *atlas_imu_sensor;
static AtlasImuStreamState atlas_imu_state;
static size_t atlas_imu_burst_words;
static uint32_t atlas_imu_burst_started_ms;
static uint32_t atlas_imu_consecutive_failures;
static volatile bool atlas_imu_transfer_done;
static volatile bool atlas_imu_transfer_error;
static AtlasImuStreamStats atlas_imu_stats;
static AtlasImuSample atlas_imu_decoded[ATLAS_IMU_STREAM_MAX_BURST_WORDS];

/**
 * @brief Bind the FIFO-mode sensor and select DMA when CubeMX linked it to SPI3.
 * @param sensor Initialized FIFO-mode LSM6DSV16B.
 * @return ATLAS_OK or a readiness error.
 */
AtlasStatus AtlasImuStream_Init(AtlasLsm6dsv16b *sensor)
{
    const SPI_HandleTypeDef *spi;
    size_t index;

    if (sensor == NULL)
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized || !sensor->config.fifo_enabled || (sensor->device.spi == NULL))
    {
        return ATLAS_ERROR_STATE;
    }
    atlas_imu_sensor = sensor;
    atlas_imu_state = ATLAS_IMU_STREAM_IDLE;
    atlas_imu_consecutive_failures = 0U;
    memset(&atlas_imu_stats, 0, sizeof(atlas_imu_stats));

    for (index = 0U; index < (sizeof(atlas_imu_tx.ecc) / sizeof(atlas_imu_tx.ecc[0])); ++index)
    {
        atlas_imu_tx.ecc[index] = 0U;
        atlas_imu_rx.ecc[index] = 0U;
    }
    atlas_imu_tx.bytes[0] = ATLAS_LSM6DSV16B_FIFO_READ_COMMAND;

    /* HAL_SPI_MspInit() (generated from Atlas.ioc) initializes and links both
     * streams; without them the identical bursts run as polled transfers. */
    spi = sensor->device.spi;
    atlas_imu_stats.dma_active = (spi->hdmarx != NULL) && (spi->hdmatx != NULL);
    return ATLAS_OK;
}

/**
 * @brief Decode a completed burst and publish its samples.
 * @param bytes First FIFO word.
 * @param words Word count.
 * @param ring Destination ring or NULL.
 * @return Samples decoded.
 */
static size_t atlas_imu_publish(const uint8_t *bytes, size_t words, AtlasSampleRing *ring)
{
    const size_t count = AtlasLsm6dsv16b_DecodeFifo(atlas_imu_sensor, bytes, words,
                                                    atlas_imu_decoded,
                                                    ATLAS_IMU_STREAM_MAX_BURST_WORDS);
    size_t index;

    atlas_imu_stats.words_read += (uint32_t)words;
    atlas_imu_stats.samples_produced += (uint32_t)count;
    if (ring != NULL)
    {
        for (index = 0U; index < count; ++index)
        {
            (void)AtlasSampleRing_Push(ring, &atlas_imu_decoded[index]);
        }
    }
    return count;
}

/**
 * @brief End a DMA burst: release chip select and clear flags.
 * @param abort true to stop an unfinished transfer first.
 */
static void atlas_imu_finish_transfer(bool abort)
{
    AtlasSpiDevice *const device = &atlas_imu_sensor->device;

    if (abort)
    {
        (void)HAL_SPI_Abort(device->spi);
    }
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
    atlas_imu_transfer_done = false;
    atlas_imu_transfer_error = false;
    atlas_imu_state = ATLAS_IMU_STREAM_IDLE;
}

/**
 * @brief Record a DMA failure and fall back to polled SPI after repeated failures.
 */
static void atlas_imu_dma_failed(void)
{
    ++atlas_imu_consecutive_failures;
    if (atlas_imu_consecutive_failures >= ATLAS_IMU_STREAM_MAX_DMA_FAILURES)
    {
        atlas_imu_stats.dma_active = false;
    }
}

/**
 * @brief Advance the drain by one non-waiting step.
 * @param ring Destination ring or NULL.
 * @param produced Optional decoded-sample count.
 * @return ATLAS_OK or a typed failure.
 */
AtlasStatus AtlasImuStream_Service(AtlasSampleRing *ring, size_t *produced)
{
    uint16_t level = 0U;
    bool overrun = false;
    AtlasStatus status;
    size_t count = 0U;

    if (produced != NULL)
    {
        *produced = 0U;
    }
    if (atlas_imu_sensor == NULL)
    {
        return ATLAS_ERROR_NOT_READY;
    }

    if (atlas_imu_state == ATLAS_IMU_STREAM_BUSY)
    {
        if (atlas_imu_transfer_error)
        {
            ++atlas_imu_stats.dma_errors;
            atlas_imu_finish_transfer(true);
            atlas_imu_dma_failed();
            return ATLAS_ERROR_IO;
        }
        if (!atlas_imu_transfer_done)
        {
            if ((uint32_t)(HAL_GetTick() - atlas_imu_burst_started_ms) >= ATLAS_IMU_STREAM_TIMEOUT_MS)
            {
                ++atlas_imu_stats.dma_timeouts;
                atlas_imu_finish_transfer(true);
                atlas_imu_dma_failed();
                return ATLAS_ERROR_TIMEOUT;
            }
            return ATLAS_OK;
        }
        atlas_imu_finish_transfer(false);
        atlas_imu_consecutive_failures = 0U;
        ++atlas_imu_stats.dma_bursts;
        count = atlas_imu_publish(&atlas_imu_rx.bytes[1], atlas_imu_burst_words, ring);
        if (produced != NULL)
        {
            *produced = count;
        }
        return ATLAS_OK;
    }

    status = AtlasLsm6dsv16b_ReadFifoStatus(atlas_imu_sensor, &level, &overrun);
    if ((status != ATLAS_OK) || (level == 0U))
    {
        return status;
    }
    if (level > atlas_imu_stats.max_fifo_words)
    {
        atlas_imu_stats.max_fifo_words = level;
    }
    /* Every FIFO timestamp is converted relative to this fresh clock pair. */
    status = AtlasLsm6dsv16b_SyncClock(atlas_imu_sensor);
    if ((status != ATLAS_OK) && (status != ATLAS_ERROR_NOT_READY))
    {
        return status;
    }
    atlas_imu_burst_words = (level > ATLAS_IMU_STREAM_MAX_BURST_WORDS) ?
                            ATLAS_IMU_STREAM_MAX_BURST_WORDS : level;

    if (atlas_imu_stats.dma_active)
    {
        AtlasSpiDevice *const device = &atlas_imu_sensor->device;
        const uint16_t length = (uint16_t)(1U + (atlas_imu_burst_words *
                                                 ATLAS_LSM6DSV16B_FIFO_WORD_BYTES));

        atlas_imu_transfer_done = false;
        atlas_imu_transfer_error = false;
        atlas_imu_burst_started_ms = HAL_GetTick();
        atlas_imu_state = ATLAS_IMU_STREAM_BUSY;
        HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
        if (HAL_SPI_TransmitReceive_DMA(device->spi, atlas_imu_tx.bytes, atlas_imu_rx.bytes,
                                        length) == HAL_OK)
        {
            return ATLAS_OK;
        }
        ++atlas_imu_stats.dma_errors;
        atlas_imu_finish_transfer(true);
        atlas_imu_dma_failed();
        return ATLAS_ERROR_IO;
    }

    status = AtlasLsm6dsv16b_ReadFifoWords(atlas_imu_sensor, &atlas_imu_rx.bytes[1],
                                           atlas_imu_burst_words);
    if (status != ATLAS_OK)
    {
        return status;
    }
    ++atlas_imu_stats.polled_bursts;
    count = atlas_imu_publish(&atlas_imu_rx.bytes[1], atlas_imu_burst_words, ring);
    if (produced != NULL)
    {
        *produced = count;
    }
    return ATLAS_OK;
}

/**
 * @brief Copy the transport counters.
 * @param stats Destination.
 */
void AtlasImuStream_GetStats(AtlasImuStreamStats *stats)
{
    if (stats != NULL)
    {
        *stats = atlas_imu_stats;
    }
}

/**
 * @brief HAL completion callback: flag only, no kernel or bus calls.
 * @param spi Completed handle.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if ((atlas_imu_sensor != NULL) && (spi == atlas_imu_sensor->device.spi))
    {
        atlas_imu_transfer_done = true;
    }
}

/**
 * @brief HAL error callback: flag only; the I/O task aborts and recovers.
 * @param spi Failed handle.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if ((atlas_imu_sensor != NULL) && (spi == atlas_imu_sensor->device.spi))
    {
        atlas_imu_transfer_error = true;
    }
}
