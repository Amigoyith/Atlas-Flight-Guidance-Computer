/**
 * @file atlas_sensor_samples.h
 * @brief Timestamped, SI-unit sensor records exchanged with estimation code.
 *
 * Major functions:
 * - None; declares the records exchanged between drivers, the RTOS and estimation.
 * - AtlasImuSample: one gyro/accel pair from the LSM6DSV16B FIFO.
 * - AtlasMagSample: one MMC5983MA field conversion.
 *
 * All vectors are in the SENSOR frame. Rotation into the vehicle body frame is an
 * estimation-layer calibration, not a driver responsibility. All timestamps use the
 * shared 1 MHz TIM2 timebase (AtlasTime_NowUs()) and wrap after ~71.6 minutes.
 */

#ifndef ATLAS_SENSOR_SAMPLES_H
#define ATLAS_SENSOR_SAMPLES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief AtlasImuSample.flags bits. */
typedef enum
{
    ATLAS_IMU_FLAG_GYRO_VALID          = (1U << 0),
    ATLAS_IMU_FLAG_ACCEL_VALID         = (1U << 1),
    /** No hardware timestamp word; time extrapolated from the prior sample. */
    ATLAS_IMU_FLAG_TIMESTAMP_ESTIMATED = (1U << 2),
    /** At least one gyro axis read the full-scale code. */
    ATLAS_IMU_FLAG_GYRO_SATURATED      = (1U << 3),
    /** At least one accel axis read the full-scale code (expected above 16 g). */
    ATLAS_IMU_FLAG_ACCEL_SATURATED     = (1U << 4),
    /** Samples were lost immediately before this one; see missed_before. */
    ATLAS_IMU_FLAG_GAP_BEFORE          = (1U << 5)
} AtlasImuSampleFlag;

/** @brief One LSM6DSV16B gyro/accel sample in SI units. */
typedef struct
{
    uint32_t timestamp_us;   /**< MCU time the sensor latched this sample. */
    uint32_t sequence;       /**< Producer count; a jump means ring overflow. */
    float gyro_rad_s[3];     /**< Sensor-frame angular rate, rad/s. */
    float accel_m_s2[3];     /**< Sensor-frame specific force, m/s^2. */
    uint16_t flags;          /**< AtlasImuSampleFlag bits. */
    uint16_t missed_before;  /**< Sensor samples lost before this one. */
} AtlasImuSample;

/** @brief AtlasMagSample.flags bits. */
typedef enum
{
    ATLAS_MAG_FLAG_VALID = (1U << 0)
} AtlasMagSampleFlag;

/** @brief One MMC5983MA field conversion in gauss. */
typedef struct
{
    uint32_t timestamp_us;  /**< MCU time at the conversion midpoint (nominal). */
    uint32_t sequence;      /**< Producer count; a jump means ring overflow. */
    float field_gauss[3];   /**< Sensor-frame field, uncalibrated for hard/soft iron. */
    uint32_t flags;         /**< AtlasMagSampleFlag bits. */
} AtlasMagSample;

#ifdef __cplusplus
}
#endif

#endif /* ATLAS_SENSOR_SAMPLES_H */
