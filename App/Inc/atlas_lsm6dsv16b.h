/**
 * @file atlas_lsm6dsv16b.h
 * @brief LSM6DSV16B low-g accelerometer and gyroscope configuration firmware.
 *
 * Major functions:
 * - AtlasLsm6dsv16b_Init(): resets, identifies, configures, and verifies the IMU.
 * - AtlasLsm6dsv16b_ReadSample(): returns coherent temperature, gyro, and accel data.
 * - AtlasLsm6dsv16b_OnInterrupt()/ConsumeInterrupt(): defer INT1 work out of ISR context.
 * - AtlasLsm6dsv16b_ReadFifoStatus()/SyncClock()/ReadFifoWords(): FIFO transport pieces.
 * - AtlasLsm6dsv16b_DecodeFifo(): turns raw tagged FIFO words into AtlasImuSample records.
 *
 * FIFO mode (flight): gyro and accel are batched at one rate together with a hardware
 * timestamp word per sample, in stream mode. Each FIFO word is 7 bytes (tag + 6 data
 * bytes); with CTRL3.IF_INC a multi-byte read starting at FIFO_DATA_OUT_TAG wraps back
 * to 0x78 after 0x7E, so any number of words can be read in one SPI burst (polled or
 * DMA). The decoder is pure: it only needs the bytes, so the transport is replaceable.
 */

#ifndef ATLAS_LSM6DSV16B_H
#define ATLAS_LSM6DSV16B_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "atlas_clock_map.h"
#include "atlas_sensor_samples.h"
#include "atlas_spi_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ATLAS_LSM6DSV16B_WHO_AM_I (0x71U)
/** Bytes per FIFO word: TAG then X/Y/Z little-endian 16-bit data. */
#define ATLAS_LSM6DSV16B_FIFO_WORD_BYTES   (7U)
/** SPI read command for FIFO_DATA_OUT_TAG (0x78) with the read bit. */
#define ATLAS_LSM6DSV16B_FIFO_READ_COMMAND (0xF8U)
/** Maximum words DIFF_FIFO can report. */
#define ATLAS_LSM6DSV16B_FIFO_MAX_WORDS    (511U)
/** Nominal timestamp LSB; the actual period is measured against the MCU clock. */
#define ATLAS_LSM6DSV16B_TIMESTAMP_US_PER_LSB (21.75F)

/** @brief Supported high-performance accelerometer/gyroscope ODR codes. */
typedef enum
{
    ATLAS_LSM6_ODR_POWER_DOWN = 0x00,
    ATLAS_LSM6_ODR_60_HZ      = 0x05,
    ATLAS_LSM6_ODR_120_HZ     = 0x06,
    ATLAS_LSM6_ODR_240_HZ     = 0x07,
    ATLAS_LSM6_ODR_480_HZ     = 0x08,
    ATLAS_LSM6_ODR_960_HZ     = 0x09
} AtlasLsm6dsv16bOdr;

/** @brief Accelerometer full-scale codes from CTRL8.FS_XL. */
typedef enum
{
    ATLAS_LSM6_ACCEL_2_G  = 0x00,
    ATLAS_LSM6_ACCEL_4_G  = 0x01,
    ATLAS_LSM6_ACCEL_8_G  = 0x02,
    ATLAS_LSM6_ACCEL_16_G = 0x03
} AtlasLsm6dsv16bAccelRange;

/** @brief Gyroscope full-scale codes from CTRL6.FS_G. */
typedef enum
{
    ATLAS_LSM6_GYRO_125_DPS  = 0x00,
    ATLAS_LSM6_GYRO_250_DPS  = 0x01,
    ATLAS_LSM6_GYRO_500_DPS  = 0x02,
    ATLAS_LSM6_GYRO_1000_DPS = 0x03,
    ATLAS_LSM6_GYRO_2000_DPS = 0x04,
    ATLAS_LSM6_GYRO_4000_DPS = 0x0C
} AtlasLsm6dsv16bGyroRange;

/** @brief Configurable baseline for the direct IMU data path. */
typedef struct
{
    AtlasLsm6dsv16bOdr accel_odr;
    AtlasLsm6dsv16bOdr gyro_odr;
    AtlasLsm6dsv16bAccelRange accel_range;
    AtlasLsm6dsv16bGyroRange gyro_range;
    bool route_data_ready_to_int1;
    /** Stream-mode FIFO with per-sample timestamps; requires accel_odr == gyro_odr. */
    bool fifo_enabled;
} AtlasLsm6dsv16bConfig;

/** @brief One coherent direct-register IMU sample. */
typedef struct
{
    int16_t raw_temperature;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    uint32_t timestamp_ms;
} AtlasLsm6dsv16bSample;

/** @brief Driver health counters. */
typedef struct
{
    uint32_t io_errors;
    uint32_t identity_failures;
    uint32_t configuration_mismatches;
    uint32_t reset_timeouts;
    volatile uint32_t interrupt_count;
    uint32_t samples_read;
    uint32_t fifo_overruns;         /**< FIFO_OVR observed: the oldest words were lost. */
    uint32_t fifo_words;            /**< Tagged words decoded. */
    uint32_t fifo_samples;          /**< AtlasImuSample records produced. */
    uint32_t samples_missed;        /**< Sensor samples inferred lost from timestamp gaps. */
    uint32_t incomplete_samples;    /**< Records missing gyro or accel. */
    uint32_t estimated_timestamps;  /**< Records without a hardware timestamp word. */
    uint32_t unknown_tags;          /**< Words with a tag this configuration never batches. */
    uint32_t output_overflows;      /**< Records discarded for lack of caller capacity. */
} AtlasLsm6dsv16bHealth;

/** @brief Partially assembled FIFO time slot (one BDR event). */
typedef struct
{
    uint32_t sensor_ticks;
    int16_t gyro[3];
    int16_t accel[3];
    uint8_t tag_count;
    bool active;
    bool has_timestamp;
    bool has_gyro;
    bool has_accel;
} AtlasLsm6dsv16bFifoSlot;

/** @brief LSM6DSV16B driver instance. */
typedef struct
{
    AtlasSpiDevice device;
    AtlasLsm6dsv16bConfig config;
    volatile uint32_t interrupt_count;
    uint32_t interrupt_count_consumed;
    bool initialized;
    AtlasLsm6dsv16bHealth health;
    /* FIFO decoding state; unused in direct-register mode. */
    AtlasClockMap clock;
    AtlasLsm6dsv16bFifoSlot slot;
    uint32_t last_sample_ticks;
    bool have_last_sample;
    bool overrun_pending;
    uint32_t next_sequence;
    /** Latest decoded FIFO sample in the legacy snapshot format. */
    AtlasLsm6dsv16bSample latest;
} AtlasLsm6dsv16b;

/**
 * @brief Return the reviewed Atlas default: 240 Hz, +/-16 g, +/-2000 dps, INT1 DRDY.
 * @return Complete default configuration.
 */
AtlasLsm6dsv16bConfig AtlasLsm6dsv16b_DefaultConfig(void);

/**
 * @brief Return the flight data path: 480 Hz FIFO with timestamps, +/-16 g, +/-4000 dps.
 * @return Complete flight configuration; INT1 data-ready routing disabled.
 */
AtlasLsm6dsv16bConfig AtlasLsm6dsv16b_FlightConfig(void);

/**
 * @brief Reset, identify, configure, and read back the LSM6DSV16B.
 * @param sensor Destination driver instance.
 * @param spi Initialized SPI3 handle in mode 3 at no more than 10 MHz.
 * @param cs_port LSM6DSV16B chip-select GPIO port.
 * @param cs_pin Active-low chip-select pin mask.
 * @param config Requested direct data-path configuration.
 * @return ATLAS_OK or a typed identity, timeout, configuration, or I/O error.
 */
AtlasStatus AtlasLsm6dsv16b_Init(AtlasLsm6dsv16b *sensor,
                                 SPI_HandleTypeDef *spi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin,
                                 const AtlasLsm6dsv16bConfig *config);

/**
 * @brief Read STATUS_REG and report whether both accel and gyro have fresh data.
 * @param sensor Initialized driver instance.
 * @param ready Destination readiness flag.
 * @return ATLAS_OK or a typed transport error.
 */
AtlasStatus AtlasLsm6dsv16b_DataReady(AtlasLsm6dsv16b *sensor, bool *ready);

/**
 * @brief Burst-read and scale temperature, gyro XYZ, and accel XYZ.
 * @param sensor Initialized driver instance.
 * @param sample Destination sample.
 * @return ATLAS_OK, ATLAS_ERROR_NOT_READY, or a typed transport error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadSample(AtlasLsm6dsv16b *sensor,
                                       AtlasLsm6dsv16bSample *sample);

/**
 * @brief Record an INT1 edge; safe to call from HAL_GPIO_EXTI_Callback.
 * @param sensor Initialized driver instance.
 */
void AtlasLsm6dsv16b_OnInterrupt(AtlasLsm6dsv16b *sensor);

/**
 * @brief Consume coalesced INT1 activity in foreground code.
 * @param sensor Initialized driver instance.
 * @return true when one or more new edges occurred since the prior call.
 */
bool AtlasLsm6dsv16b_ConsumeInterrupt(AtlasLsm6dsv16b *sensor);

/**
 * @brief Read the FIFO fill level and overrun flag.
 * @param sensor Initialized FIFO-mode instance.
 * @param words Destination count of unread 7-byte words.
 * @param overrun Destination; true when words were lost since the prior status read.
 * @return ATLAS_OK, ATLAS_ERROR_STATE outside FIFO mode, or a transport error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadFifoStatus(AtlasLsm6dsv16b *sensor,
                                           uint16_t *words,
                                           bool *overrun);

/**
 * @brief Read TIMESTAMP0..3 bracketed by the MCU clock and feed the clock map.
 * @param sensor Initialized FIFO-mode instance.
 * @return ATLAS_OK, ATLAS_ERROR_NOT_READY without a microsecond timebase, or an error.
 * @note Call roughly once per drain; samples are timed relative to the latest pair.
 */
AtlasStatus AtlasLsm6dsv16b_SyncClock(AtlasLsm6dsv16b *sensor);

/**
 * @brief Read FIFO words with polled SPI bursts (fallback for the DMA path).
 * @param sensor Initialized FIFO-mode instance.
 * @param bytes Destination of words * 7 bytes.
 * @param words Word count, at most ATLAS_LSM6DSV16B_FIFO_MAX_WORDS.
 * @return ATLAS_OK or a transport error; bytes are undefined on error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadFifoWords(AtlasLsm6dsv16b *sensor,
                                          uint8_t *bytes,
                                          size_t words);

/**
 * @brief Decode raw FIFO words into timestamped SI samples.
 * @param sensor Initialized FIFO-mode instance; carries slot state across calls.
 * @param bytes words * 7 bytes read from FIFO_DATA_OUT_TAG onward.
 * @param words Number of words in bytes.
 * @param samples Destination records.
 * @param capacity Record capacity; capacity >= words never discards.
 * @return Records written. A slot split across two reads completes on the later call.
 * @note Pure computation apart from updating sensor state and counters.
 */
size_t AtlasLsm6dsv16b_DecodeFifo(AtlasLsm6dsv16b *sensor,
                                  const uint8_t *bytes,
                                  size_t words,
                                  AtlasImuSample *samples,
                                  size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* ATLAS_LSM6DSV16B_H */
