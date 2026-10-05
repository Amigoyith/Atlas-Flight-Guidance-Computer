/**
 * @file atlas_lsm6dsv16b.c
 * @brief Verified LSM6DSV16B mode-3 SPI setup and direct-register sampling.
 *
 * Major functions:
 * - AtlasLsm6dsv16b_Init(): performs software reset, identity check, and readback.
 * - AtlasLsm6dsv16b_ReadSample(): handles the device's unusual Z/Y/X accel map.
 * - AtlasLsm6dsv16b_OnInterrupt(): records an edge without bus work in the ISR.
 * - AtlasLsm6dsv16b_DecodeFifo(): groups tagged words into timestamped samples.
 *
 * Register facts are taken from ST's lsm6dsv16b-pid register driver; the FIFO burst
 * pattern matches the Linux st_lsm6dsx tagged-FIFO reader.
 */

#include "atlas_lsm6dsv16b.h"

#include "atlas_time.h"

#include <string.h>

#define LSM6_REG_FIFO_CTRL3       (0x09U)
#define LSM6_REG_FIFO_CTRL4       (0x0AU)
#define LSM6_REG_INT1_CTRL        (0x0DU)
#define LSM6_REG_WHO_AM_I         (0x0FU)
#define LSM6_REG_CTRL1            (0x10U)
#define LSM6_REG_CTRL2            (0x11U)
#define LSM6_REG_CTRL3            (0x12U)
#define LSM6_REG_CTRL6            (0x15U)
#define LSM6_REG_CTRL8            (0x17U)
#define LSM6_REG_FIFO_STATUS1     (0x1BU)
#define LSM6_REG_STATUS           (0x1EU)
#define LSM6_REG_OUT_TEMP_L       (0x20U)
#define LSM6_REG_TIMESTAMP0       (0x40U)
#define LSM6_REG_FUNCTIONS_ENABLE (0x50U)

#define LSM6_SPI_READ       (0x80U)
#define LSM6_CTRL3_BDU      (0x40U)
#define LSM6_CTRL3_IF_INC   (0x04U)
#define LSM6_CTRL3_SW_RESET (0x01U)
#define LSM6_STATUS_GDA     (0x02U)
#define LSM6_STATUS_XLDA    (0x01U)
#define LSM6_INT1_GYRO_DRDY (0x02U)
#define LSM6_INT1_XL_DRDY   (0x01U)

/* FIFO_CTRL4: DEC_TS_BATCH=1 (timestamp every BDR event), ODR_T_BATCH=1 (1.875 Hz
 * temperature for gyro-bias modelling), FIFO_MODE=110b continuous stream. */
#define LSM6_FIFO_CTRL4_FLIGHT    (0x56U)
#define LSM6_FIFO_CTRL4_MASK      (0xF7U)
#define LSM6_FUNCTIONS_TIMESTAMP  (0x40U)
#define LSM6_FIFO_STATUS2_OVR_IA  (0x40U)
#define LSM6_FIFO_STATUS2_OVR_LATCHED (0x08U)
#define LSM6_FIFO_STATUS2_DIFF_MSB (0x01U)

#define LSM6_TAG_GYRO        (0x01U)
#define LSM6_TAG_ACCEL       (0x02U)
#define LSM6_TAG_TEMPERATURE (0x03U)
#define LSM6_TAG_TIMESTAMP   (0x04U)
#define LSM6_TAG_CFG_CHANGE  (0x05U)

/** Polled bursts are chunked to bound stack use; each chunk re-issues 0xF8. */
#define LSM6_POLLED_BURST_WORDS (8U)

#define LSM6_DEG_TO_RAD      (0.017453292519943295F)
#define LSM6_STANDARD_GRAVITY (9.80665F)

/**
 * @brief Read adjacent LSM6DSV16B registers through SPI.
 * @param sensor Driver instance.
 * @param reg First register address.
 * @param data Destination buffer.
 * @param length Number of bytes; range 1..14 for current uses.
 * @return ATLAS_OK or typed transport error.
 */
static AtlasStatus atlas_lsm6_read(AtlasLsm6dsv16b *sensor,
                                   uint8_t reg,
                                   uint8_t *data,
                                   size_t length)
{
    uint8_t tx[15] = {0U};
    uint8_t rx[15] = {0U};
    AtlasStatus status;

    if ((length == 0U) || (length > 14U))
    {
        return ATLAS_ERROR_ARGUMENT;
    }
    tx[0] = (uint8_t)(LSM6_SPI_READ | reg);
    status = AtlasSpiDevice_Transfer(&sensor->device, tx, rx, length + 1U);
    if (status != ATLAS_OK)
    {
        ++sensor->health.io_errors;
        return status;
    }
    memcpy(data, &rx[1], length);
    return ATLAS_OK;
}

/**
 * @brief Write one LSM6DSV16B register.
 * @param sensor Driver instance.
 * @param reg Register address.
 * @param value Register value.
 * @return ATLAS_OK or typed transport error.
 */
static AtlasStatus atlas_lsm6_write(AtlasLsm6dsv16b *sensor,
                                    uint8_t reg,
                                    uint8_t value)
{
    const uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};
    uint8_t rx[2] = {0U};
    AtlasStatus status = AtlasSpiDevice_Transfer(&sensor->device, tx, rx, sizeof(tx));

    if (status != ATLAS_OK)
    {
        ++sensor->health.io_errors;
    }
    return status;
}

/**
 * @brief Write and verify one configuration register under a comparison mask.
 * @param sensor Driver instance.
 * @param reg Register address.
 * @param value New value.
 * @param mask Bits that must read back identically.
 * @return ATLAS_OK, typed transport error, or protocol mismatch.
 */
static AtlasStatus atlas_lsm6_write_checked(AtlasLsm6dsv16b *sensor,
                                            uint8_t reg,
                                            uint8_t value,
                                            uint8_t mask)
{
    uint8_t actual = 0U;
    AtlasStatus status = atlas_lsm6_write(sensor, reg, value);

    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_read(sensor, reg, &actual, 1U);
    }
    if ((status == ATLAS_OK) && ((actual & mask) != (value & mask)))
    {
        ++sensor->health.configuration_mismatches;
        return ATLAS_ERROR_PROTOCOL;
    }
    return status;
}

/**
 * @brief Decode a little-endian signed 16-bit register pair.
 * @param bytes Pointer to the least-significant byte.
 * @return Signed sample.
 */
static int16_t atlas_lsm6_i16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

/**
 * @brief Return accelerometer sensitivity for the configured full scale.
 * @param range Full-scale code.
 * @return Nominal g per LSB.
 */
static float atlas_lsm6_accel_scale(AtlasLsm6dsv16bAccelRange range)
{
    switch (range)
    {
        case ATLAS_LSM6_ACCEL_2_G:  return 0.000061F;
        case ATLAS_LSM6_ACCEL_4_G:  return 0.000122F;
        case ATLAS_LSM6_ACCEL_8_G:  return 0.000244F;
        case ATLAS_LSM6_ACCEL_16_G: return 0.000488F;
        default:                    return 0.0F;
    }
}

/**
 * @brief Return gyroscope sensitivity for the configured full scale.
 * @param range Full-scale code.
 * @return Nominal degrees/second per LSB.
 */
static float atlas_lsm6_gyro_scale(AtlasLsm6dsv16bGyroRange range)
{
    switch (range)
    {
        case ATLAS_LSM6_GYRO_125_DPS:  return 0.004375F;
        case ATLAS_LSM6_GYRO_250_DPS:  return 0.00875F;
        case ATLAS_LSM6_GYRO_500_DPS:  return 0.0175F;
        case ATLAS_LSM6_GYRO_1000_DPS: return 0.035F;
        case ATLAS_LSM6_GYRO_2000_DPS: return 0.070F;
        case ATLAS_LSM6_GYRO_4000_DPS: return 0.140F;
        default:                       return 0.0F;
    }
}

/**
 * @brief Validate a requested direct-data-path configuration.
 * @param config Configuration to inspect.
 * @return true when all encoded values are supported.
 */
static bool atlas_lsm6_config_valid(const AtlasLsm6dsv16bConfig *config)
{
    if (config == NULL)
    {
        return false;
    }
    if ((config->accel_odr < ATLAS_LSM6_ODR_60_HZ) ||
        (config->accel_odr > ATLAS_LSM6_ODR_960_HZ) ||
        (config->gyro_odr < ATLAS_LSM6_ODR_60_HZ) ||
        (config->gyro_odr > ATLAS_LSM6_ODR_960_HZ))
    {
        return false;
    }
    /* Slot assembly pairs one gyro and one accel word per BDR event. */
    if (config->fifo_enabled && (config->accel_odr != config->gyro_odr))
    {
        return false;
    }
    return (atlas_lsm6_accel_scale(config->accel_range) > 0.0F) &&
           (atlas_lsm6_gyro_scale(config->gyro_range) > 0.0F);
}

/**
 * @brief Return the reviewed Atlas default: 240 Hz, +/-16 g, +/-2000 dps, INT1 DRDY.
 * @return Complete default configuration.
 */
AtlasLsm6dsv16bConfig AtlasLsm6dsv16b_DefaultConfig(void)
{
    AtlasLsm6dsv16bConfig config;

    config.accel_odr = ATLAS_LSM6_ODR_240_HZ;
    config.gyro_odr = ATLAS_LSM6_ODR_240_HZ;
    config.accel_range = ATLAS_LSM6_ACCEL_16_G;
    config.gyro_range = ATLAS_LSM6_GYRO_2000_DPS;
    config.route_data_ready_to_int1 = true;
    config.fifo_enabled = false;
    return config;
}

/**
 * @brief Return the flight data path: 480 Hz FIFO with timestamps, +/-16 g, +/-4000 dps.
 * @return Complete flight configuration.
 */
AtlasLsm6dsv16bConfig AtlasLsm6dsv16b_FlightConfig(void)
{
    AtlasLsm6dsv16bConfig config;

    config.accel_odr = ATLAS_LSM6_ODR_480_HZ;
    config.gyro_odr = ATLAS_LSM6_ODR_480_HZ;
    config.accel_range = ATLAS_LSM6_ACCEL_16_G;
    config.gyro_range = ATLAS_LSM6_GYRO_4000_DPS;
    config.route_data_ready_to_int1 = false;
    config.fifo_enabled = true;
    return config;
}

/**
 * @brief Return the nominal output data rate of an ODR code.
 * @param odr Validated ODR code.
 * @return Rate in hertz.
 */
static float atlas_lsm6_odr_hz(AtlasLsm6dsv16bOdr odr)
{
    switch (odr)
    {
        case ATLAS_LSM6_ODR_60_HZ:  return 60.0F;
        case ATLAS_LSM6_ODR_120_HZ: return 120.0F;
        case ATLAS_LSM6_ODR_240_HZ: return 240.0F;
        case ATLAS_LSM6_ODR_480_HZ: return 480.0F;
        case ATLAS_LSM6_ODR_960_HZ: return 960.0F;
        default:                    return 0.0F;
    }
}

/**
 * @brief Configure stream-mode FIFO batching of gyro, accel, timestamp and temperature.
 * @param sensor Driver instance after the direct-path registers are verified.
 * @param config Validated FIFO configuration.
 * @return ATLAS_OK or a transport/readback failure.
 */
static AtlasStatus atlas_lsm6_configure_fifo(AtlasLsm6dsv16b *sensor,
                                             const AtlasLsm6dsv16bConfig *config)
{
    uint8_t functions = 0U;
    /* BDR codes use the same encoding as the ODR codes. */
    AtlasStatus status = atlas_lsm6_write_checked(sensor,
        LSM6_REG_FIFO_CTRL3,
        (uint8_t)(((uint8_t)config->gyro_odr << 4) | (uint8_t)config->accel_odr),
        0xFFU);

    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_read(sensor, LSM6_REG_FUNCTIONS_ENABLE, &functions, 1U);
    }
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_FUNCTIONS_ENABLE,
                                          (uint8_t)(functions | LSM6_FUNCTIONS_TIMESTAMP),
                                          LSM6_FUNCTIONS_TIMESTAMP);
    }
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_FIFO_CTRL4,
                                          LSM6_FIFO_CTRL4_FLIGHT,
                                          LSM6_FIFO_CTRL4_MASK);
    }
    return status;
}

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
                                 const AtlasLsm6dsv16bConfig *config)
{
    uint8_t value = 0U;
    uint32_t started_ms;
    AtlasStatus status;

    if ((sensor == NULL) || (spi == NULL) || (cs_port == NULL) || (config == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if (!atlas_lsm6_config_valid(config))
    {
        return ATLAS_ERROR_ARGUMENT;
    }

    memset(sensor, 0, sizeof(*sensor));
    status = AtlasSpiDevice_Init(&sensor->device, spi, cs_port, cs_pin, 20U);
    if (status != ATLAS_OK)
    {
        return status;
    }

    status = atlas_lsm6_read(sensor, LSM6_REG_WHO_AM_I, &value, 1U);
    if (status != ATLAS_OK)
    {
        return status;
    }
    if (value != ATLAS_LSM6DSV16B_WHO_AM_I)
    {
        ++sensor->health.identity_failures;
        return ATLAS_ERROR_IDENTITY;
    }

    status = atlas_lsm6_write(sensor, LSM6_REG_CTRL3, LSM6_CTRL3_SW_RESET);
    started_ms = HAL_GetTick();
    while (status == ATLAS_OK)
    {
        status = atlas_lsm6_read(sensor, LSM6_REG_CTRL3, &value, 1U);
        if ((status == ATLAS_OK) && ((value & LSM6_CTRL3_SW_RESET) == 0U))
        {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started_ms) >= 50U)
        {
            ++sensor->health.reset_timeouts;
            return ATLAS_ERROR_TIMEOUT;
        }
        AtlasTime_DelayMs(1U);
    }
    if (status != ATLAS_OK)
    {
        return status;
    }

    /* BDU prevents torn words; IF_INC enables the verified 14-byte burst below. */
    status = atlas_lsm6_write_checked(sensor,
                                      LSM6_REG_CTRL3,
                                      LSM6_CTRL3_BDU | LSM6_CTRL3_IF_INC,
                                      LSM6_CTRL3_BDU | LSM6_CTRL3_IF_INC);
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_CTRL8,
                                          (uint8_t)config->accel_range,
                                          0x03U);
    }
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_CTRL6,
                                          (uint8_t)config->gyro_range,
                                          0x0FU);
    }
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_CTRL1,
                                          (uint8_t)config->accel_odr,
                                          0x7FU);
    }
    if (status == ATLAS_OK)
    {
        status = atlas_lsm6_write_checked(sensor,
                                          LSM6_REG_CTRL2,
                                          (uint8_t)config->gyro_odr,
                                          0x7FU);
    }
    if (status == ATLAS_OK)
    {
        const uint8_t int1 = config->route_data_ready_to_int1 ?
                             (LSM6_INT1_GYRO_DRDY | LSM6_INT1_XL_DRDY) : 0U;
        status = atlas_lsm6_write_checked(sensor, LSM6_REG_INT1_CTRL, int1, 0x03U);
    }
    if ((status == ATLAS_OK) && config->fifo_enabled)
    {
        status = atlas_lsm6_configure_fifo(sensor, config);
    }
    if (status != ATLAS_OK)
    {
        return status;
    }

    AtlasClockMap_Init(&sensor->clock, ATLAS_LSM6DSV16B_TIMESTAMP_US_PER_LSB);
    sensor->config = *config;
    sensor->initialized = true;
    return ATLAS_OK;
}

/**
 * @brief Read STATUS_REG and report whether both accel and gyro have fresh data.
 * @param sensor Initialized driver instance.
 * @param ready Destination readiness flag.
 * @return ATLAS_OK or a typed transport error.
 */
AtlasStatus AtlasLsm6dsv16b_DataReady(AtlasLsm6dsv16b *sensor, bool *ready)
{
    uint8_t status_reg = 0U;
    AtlasStatus status;

    if ((sensor == NULL) || (ready == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized)
    {
        return ATLAS_ERROR_NOT_READY;
    }
    status = atlas_lsm6_read(sensor, LSM6_REG_STATUS, &status_reg, 1U);
    if (status == ATLAS_OK)
    {
        *ready = ((status_reg & (LSM6_STATUS_GDA | LSM6_STATUS_XLDA)) ==
                  (LSM6_STATUS_GDA | LSM6_STATUS_XLDA));
    }
    return status;
}

/**
 * @brief Burst-read and scale temperature, gyro XYZ, and accel XYZ.
 * @param sensor Initialized driver instance.
 * @param sample Destination sample.
 * @return ATLAS_OK, ATLAS_ERROR_NOT_READY, or a typed transport error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadSample(AtlasLsm6dsv16b *sensor,
                                       AtlasLsm6dsv16bSample *sample)
{
    uint8_t bytes[14];
    bool ready = false;
    float accel_scale;
    float gyro_scale;
    AtlasStatus status;

    if ((sensor == NULL) || (sample == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized)
    {
        return ATLAS_ERROR_NOT_READY;
    }

    status = AtlasLsm6dsv16b_DataReady(sensor, &ready);
    if (status != ATLAS_OK)
    {
        return status;
    }
    if (!ready)
    {
        return ATLAS_ERROR_NOT_READY;
    }

    status = atlas_lsm6_read(sensor, LSM6_REG_OUT_TEMP_L, bytes, sizeof(bytes));
    if (status != ATLAS_OK)
    {
        return status;
    }

    sample->raw_temperature = atlas_lsm6_i16(&bytes[0]);
    sample->raw_gyro_x = atlas_lsm6_i16(&bytes[2]);
    sample->raw_gyro_y = atlas_lsm6_i16(&bytes[4]);
    sample->raw_gyro_z = atlas_lsm6_i16(&bytes[6]);
    /* This part maps the direct accel registers as Z, Y, X at 0x28..0x2D. */
    sample->raw_accel_z = atlas_lsm6_i16(&bytes[8]);
    sample->raw_accel_y = atlas_lsm6_i16(&bytes[10]);
    sample->raw_accel_x = atlas_lsm6_i16(&bytes[12]);

    accel_scale = atlas_lsm6_accel_scale(sensor->config.accel_range);
    gyro_scale = atlas_lsm6_gyro_scale(sensor->config.gyro_range);
    sample->temperature_c = 25.0F + ((float)sample->raw_temperature / 256.0F);
    sample->gyro_x_dps = (float)sample->raw_gyro_x * gyro_scale;
    sample->gyro_y_dps = (float)sample->raw_gyro_y * gyro_scale;
    sample->gyro_z_dps = (float)sample->raw_gyro_z * gyro_scale;
    sample->accel_x_g = (float)sample->raw_accel_x * accel_scale;
    sample->accel_y_g = (float)sample->raw_accel_y * accel_scale;
    sample->accel_z_g = (float)sample->raw_accel_z * accel_scale;
    sample->timestamp_ms = HAL_GetTick();
    ++sensor->health.samples_read;
    return ATLAS_OK;
}

/**
 * @brief Record an INT1 edge; safe to call from HAL_GPIO_EXTI_Callback.
 * @param sensor Initialized driver instance.
 */
void AtlasLsm6dsv16b_OnInterrupt(AtlasLsm6dsv16b *sensor)
{
    if (sensor != NULL)
    {
        ++sensor->interrupt_count;
        ++sensor->health.interrupt_count;
        __DMB();
    }
}

/**
 * @brief Consume coalesced INT1 activity in foreground code.
 * @param sensor Initialized driver instance.
 * @return true when one or more new edges occurred since the prior call.
 */
bool AtlasLsm6dsv16b_ConsumeInterrupt(AtlasLsm6dsv16b *sensor)
{
    uint32_t count;

    if (sensor == NULL)
    {
        return false;
    }
    count = sensor->interrupt_count;
    if (count == sensor->interrupt_count_consumed)
    {
        return false;
    }
    sensor->interrupt_count_consumed = count;
    return true;
}

/**
 * @brief Read the FIFO fill level and overrun flag.
 * @param sensor Initialized FIFO-mode instance.
 * @param words Destination unread-word count.
 * @param overrun Destination overrun indication.
 * @return ATLAS_OK, ATLAS_ERROR_STATE outside FIFO mode, or a transport error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadFifoStatus(AtlasLsm6dsv16b *sensor,
                                           uint16_t *words,
                                           bool *overrun)
{
    uint8_t status_bytes[2] = {0U};
    AtlasStatus status;

    if ((sensor == NULL) || (words == NULL) || (overrun == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized)
    {
        return ATLAS_ERROR_NOT_READY;
    }
    if (!sensor->config.fifo_enabled)
    {
        return ATLAS_ERROR_STATE;
    }
    status = atlas_lsm6_read(sensor, LSM6_REG_FIFO_STATUS1, status_bytes, sizeof(status_bytes));
    if (status != ATLAS_OK)
    {
        return status;
    }
    *words = (uint16_t)(status_bytes[0] |
                        ((uint16_t)(status_bytes[1] & LSM6_FIFO_STATUS2_DIFF_MSB) << 8));
    *overrun = (status_bytes[1] &
                (LSM6_FIFO_STATUS2_OVR_IA | LSM6_FIFO_STATUS2_OVR_LATCHED)) != 0U;
    if (*overrun)
    {
        ++sensor->health.fifo_overruns;
        sensor->overrun_pending = true;
    }
    return ATLAS_OK;
}

/**
 * @brief Pair the sensor timestamp counter with the MCU microsecond clock.
 * @param sensor Initialized FIFO-mode instance.
 * @return ATLAS_OK or a readiness/transport error.
 */
AtlasStatus AtlasLsm6dsv16b_SyncClock(AtlasLsm6dsv16b *sensor)
{
    uint8_t bytes[4] = {0U};
    uint32_t before_us;
    uint32_t after_us;
    AtlasStatus status;

    if (sensor == NULL)
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized || !AtlasTime_MicrosecondsAvailable())
    {
        return ATLAS_ERROR_NOT_READY;
    }
    if (!sensor->config.fifo_enabled)
    {
        return ATLAS_ERROR_STATE;
    }
    before_us = AtlasTime_NowUs();
    status = atlas_lsm6_read(sensor, LSM6_REG_TIMESTAMP0, bytes, sizeof(bytes));
    after_us = AtlasTime_NowUs();
    if (status != ATLAS_OK)
    {
        return status;
    }
    /* The counter is latched mid-transfer; the bracket midpoint bounds the error to
     * half the transaction time (about 10 us at the configured SPI clock). */
    AtlasClockMap_AddPair(&sensor->clock,
                          (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                          ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24),
                          before_us + ((after_us - before_us) / 2U));
    return ATLAS_OK;
}

/**
 * @brief Read FIFO words using chunked polled bursts from FIFO_DATA_OUT_TAG.
 * @param sensor Initialized FIFO-mode instance.
 * @param bytes Destination of words * 7 bytes.
 * @param words Word count.
 * @return ATLAS_OK or a transport error.
 */
AtlasStatus AtlasLsm6dsv16b_ReadFifoWords(AtlasLsm6dsv16b *sensor,
                                          uint8_t *bytes,
                                          size_t words)
{
    uint8_t tx[1U + (LSM6_POLLED_BURST_WORDS * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES)];
    uint8_t rx[sizeof(tx)];
    size_t done = 0U;

    if ((sensor == NULL) || (bytes == NULL))
    {
        return ATLAS_ERROR_NULL;
    }
    if (!sensor->initialized)
    {
        return ATLAS_ERROR_NOT_READY;
    }
    if (!sensor->config.fifo_enabled)
    {
        return ATLAS_ERROR_STATE;
    }
    if (words > ATLAS_LSM6DSV16B_FIFO_MAX_WORDS)
    {
        return ATLAS_ERROR_ARGUMENT;
    }
    memset(tx, 0, sizeof(tx));
    tx[0] = ATLAS_LSM6DSV16B_FIFO_READ_COMMAND;
    while (done < words)
    {
        const size_t chunk = ((words - done) > LSM6_POLLED_BURST_WORDS) ?
                             LSM6_POLLED_BURST_WORDS : (words - done);
        const size_t length = chunk * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES;
        const AtlasStatus status = AtlasSpiDevice_Transfer(&sensor->device, tx, rx, length + 1U);
        if (status != ATLAS_OK)
        {
            ++sensor->health.io_errors;
            return status;
        }
        memcpy(&bytes[done * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES], &rx[1], length);
        done += chunk;
    }
    return ATLAS_OK;
}

/**
 * @brief Report whether a raw axis value is a full-scale (saturated) code.
 * @param raw Signed 16-bit sample.
 * @return true at either rail.
 */
static bool atlas_lsm6_saturated(int16_t raw)
{
    return (raw == INT16_MAX) || (raw == INT16_MIN);
}

/**
 * @brief Convert the pending slot into one AtlasImuSample and clear the slot.
 * @param sensor FIFO-mode instance.
 * @param samples Destination array.
 * @param capacity Destination capacity.
 * @param count In/out records written.
 * @note A slot holding only a timestamp is left untouched for the data that follows.
 */
static void atlas_lsm6_emit_slot(AtlasLsm6dsv16b *sensor,
                                 AtlasImuSample *samples,
                                 size_t capacity,
                                 size_t *count)
{
    AtlasLsm6dsv16bFifoSlot *const slot = &sensor->slot;
    const float gyro_scale = atlas_lsm6_gyro_scale(sensor->config.gyro_range);
    const float accel_scale = atlas_lsm6_accel_scale(sensor->config.accel_range);
    const float period_us = 1000000.0F / atlas_lsm6_odr_hz(sensor->config.gyro_odr);
    const float period_ticks = period_us / sensor->clock.us_per_tick;
    AtlasImuSample sample;
    uint32_t ticks;
    uint16_t flags = 0U;
    uint16_t missed = 0U;
    unsigned axis;

    if (!slot->active || (!slot->has_gyro && !slot->has_accel))
    {
        return;
    }

    if (slot->has_timestamp)
    {
        ticks = slot->sensor_ticks;
    }
    else if (sensor->have_last_sample)
    {
        ticks = sensor->last_sample_ticks + (uint32_t)(period_ticks + 0.5F);
        flags |= ATLAS_IMU_FLAG_TIMESTAMP_ESTIMATED;
        ++sensor->health.estimated_timestamps;
    }
    else
    {
        ticks = sensor->clock.anchor_ticks;
        flags |= ATLAS_IMU_FLAG_TIMESTAMP_ESTIMATED;
        ++sensor->health.estimated_timestamps;
    }

    if (sensor->have_last_sample && slot->has_timestamp)
    {
        const float intervals = (float)(uint32_t)(ticks - sensor->last_sample_ticks) /
                                period_ticks;
        if (intervals >= 1.5F)
        {
            /* Round (intervals - 1) to the nearest whole number of lost samples. */
            const float lost = intervals - 0.5F;
            missed = (lost >= 65535.0F) ? 65535U : (uint16_t)lost;
        }
    }
    if ((missed != 0U) || sensor->overrun_pending)
    {
        flags |= ATLAS_IMU_FLAG_GAP_BEFORE;
        sensor->health.samples_missed += missed;
    }
    sensor->overrun_pending = false;

    memset(&sample, 0, sizeof(sample));
    if (!AtlasClockMap_ToMcuUs(&sensor->clock, ticks, &sample.timestamp_us))
    {
        sample.timestamp_us = AtlasTime_NowUs();
        flags |= ATLAS_IMU_FLAG_TIMESTAMP_ESTIMATED;
    }
    sample.sequence = sensor->next_sequence++;
    if (slot->has_gyro)
    {
        flags |= ATLAS_IMU_FLAG_GYRO_VALID;
        for (axis = 0U; axis < 3U; ++axis)
        {
            sample.gyro_rad_s[axis] = (float)slot->gyro[axis] * gyro_scale * LSM6_DEG_TO_RAD;
            if (atlas_lsm6_saturated(slot->gyro[axis]))
            {
                flags |= ATLAS_IMU_FLAG_GYRO_SATURATED;
            }
        }
        sensor->latest.raw_gyro_x = slot->gyro[0];
        sensor->latest.raw_gyro_y = slot->gyro[1];
        sensor->latest.raw_gyro_z = slot->gyro[2];
        sensor->latest.gyro_x_dps = (float)slot->gyro[0] * gyro_scale;
        sensor->latest.gyro_y_dps = (float)slot->gyro[1] * gyro_scale;
        sensor->latest.gyro_z_dps = (float)slot->gyro[2] * gyro_scale;
    }
    if (slot->has_accel)
    {
        flags |= ATLAS_IMU_FLAG_ACCEL_VALID;
        for (axis = 0U; axis < 3U; ++axis)
        {
            sample.accel_m_s2[axis] = (float)slot->accel[axis] * accel_scale *
                                      LSM6_STANDARD_GRAVITY;
            if (atlas_lsm6_saturated(slot->accel[axis]))
            {
                flags |= ATLAS_IMU_FLAG_ACCEL_SATURATED;
            }
        }
        sensor->latest.raw_accel_x = slot->accel[0];
        sensor->latest.raw_accel_y = slot->accel[1];
        sensor->latest.raw_accel_z = slot->accel[2];
        sensor->latest.accel_x_g = (float)slot->accel[0] * accel_scale;
        sensor->latest.accel_y_g = (float)slot->accel[1] * accel_scale;
        sensor->latest.accel_z_g = (float)slot->accel[2] * accel_scale;
    }
    if (!slot->has_gyro || !slot->has_accel)
    {
        ++sensor->health.incomplete_samples;
    }
    sample.flags = flags;
    sample.missed_before = missed;
    sensor->latest.timestamp_ms = HAL_GetTick();

    sensor->last_sample_ticks = ticks;
    sensor->have_last_sample = true;
    memset(slot, 0, sizeof(*slot));

    if (*count < capacity)
    {
        samples[*count] = sample;
        ++*count;
        ++sensor->health.fifo_samples;
        ++sensor->health.samples_read;
    }
    else
    {
        ++sensor->health.output_overflows;
    }
}

/**
 * @brief Start a new slot for a word belonging to TAG_CNT tag_count.
 * @param slot Slot to reset.
 * @param tag_count Two-bit time-slot counter from the word's tag byte.
 */
static void atlas_lsm6_open_slot(AtlasLsm6dsv16bFifoSlot *slot, uint8_t tag_count)
{
    memset(slot, 0, sizeof(*slot));
    slot->active = true;
    slot->tag_count = tag_count;
}

/**
 * @brief Decode raw FIFO words into timestamped SI samples.
 * @param sensor FIFO-mode instance carrying slot state across calls.
 * @param bytes Raw words.
 * @param words Word count.
 * @param samples Destination records.
 * @param capacity Destination capacity.
 * @return Records written.
 */
size_t AtlasLsm6dsv16b_DecodeFifo(AtlasLsm6dsv16b *sensor,
                                  const uint8_t *bytes,
                                  size_t words,
                                  AtlasImuSample *samples,
                                  size_t capacity)
{
    size_t count = 0U;
    size_t index;

    if ((sensor == NULL) || (bytes == NULL) || ((samples == NULL) && (capacity != 0U)))
    {
        return 0U;
    }
    for (index = 0U; index < words; ++index)
    {
        const uint8_t *const word = &bytes[index * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
        const uint8_t tag = (uint8_t)(word[0] >> 3);
        const uint8_t tag_count = (uint8_t)((word[0] >> 1) & 0x03U);
        AtlasLsm6dsv16bFifoSlot *const slot = &sensor->slot;

        ++sensor->health.fifo_words;
        switch (tag)
        {
            case LSM6_TAG_TIMESTAMP:
            {
                const uint32_t ticks = (uint32_t)word[1] | ((uint32_t)word[2] << 8) |
                                       ((uint32_t)word[3] << 16) | ((uint32_t)word[4] << 24);
                /* A timestamp for the same time slot completes data that preceded it. */
                if (slot->active && !slot->has_timestamp && (slot->tag_count == tag_count) &&
                    (slot->has_gyro || slot->has_accel))
                {
                    slot->sensor_ticks = ticks;
                    slot->has_timestamp = true;
                    if (slot->has_gyro && slot->has_accel)
                    {
                        atlas_lsm6_emit_slot(sensor, samples, capacity, &count);
                    }
                    break;
                }
                atlas_lsm6_emit_slot(sensor, samples, capacity, &count);
                atlas_lsm6_open_slot(slot, tag_count);
                slot->sensor_ticks = ticks;
                slot->has_timestamp = true;
                break;
            }
            case LSM6_TAG_GYRO:
            case LSM6_TAG_ACCEL:
            {
                const bool is_gyro = (tag == LSM6_TAG_GYRO);
                if (slot->active &&
                    ((slot->tag_count != tag_count) ||
                     (is_gyro ? slot->has_gyro : slot->has_accel)))
                {
                    atlas_lsm6_emit_slot(sensor, samples, capacity, &count);
                    /* A lone timestamp from a different time slot does not apply here. */
                    atlas_lsm6_open_slot(slot, tag_count);
                }
                if (!slot->active)
                {
                    atlas_lsm6_open_slot(slot, tag_count);
                }
                if (is_gyro)
                {
                    slot->gyro[0] = atlas_lsm6_i16(&word[1]);
                    slot->gyro[1] = atlas_lsm6_i16(&word[3]);
                    slot->gyro[2] = atlas_lsm6_i16(&word[5]);
                    slot->has_gyro = true;
                }
                else
                {
                    slot->accel[0] = atlas_lsm6_i16(&word[1]);
                    slot->accel[1] = atlas_lsm6_i16(&word[3]);
                    slot->accel[2] = atlas_lsm6_i16(&word[5]);
                    slot->has_accel = true;
                }
                /* Emit at once when complete; data still lacking its timestamp waits
                 * for the next word, which may be the matching timestamp. */
                if (slot->has_gyro && slot->has_accel && slot->has_timestamp)
                {
                    atlas_lsm6_emit_slot(sensor, samples, capacity, &count);
                }
                break;
            }
            case LSM6_TAG_TEMPERATURE:
                sensor->latest.raw_temperature = atlas_lsm6_i16(&word[1]);
                sensor->latest.temperature_c =
                    25.0F + ((float)sensor->latest.raw_temperature / 256.0F);
                break;
            case LSM6_TAG_CFG_CHANGE:
                break;
            default:
                ++sensor->health.unknown_tags;
                break;
        }
    }
    return count;
}
