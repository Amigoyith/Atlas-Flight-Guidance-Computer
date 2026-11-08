/**
 * @file atlas_flight_estimator.h
 * @brief Flight attitude estimator: Fusion AHRS (ENU) on the LSM6DSV16B FIFO stream and
 *        the MMC5983MA, with modes switched by the flight state machine.
 *
 * - CALIBRATE (PAD): gyro + accelerometer + magnetometer. Gyro bias is averaged while still.
 * - GYRO_MAG (ENGINE_ON, COAST, CHUTE): gyro + magnetometer only. The accelerometer is
 *   never used for attitude: in boost it saturates and in coast it reads drag, not gravity.
 *
 * Every IMU sample is fused with its own measured interval (microsecond sensor timestamps),
 * so the attitude does not depend on how often the 100 Hz application task runs.
 * Separately, it single-integrates ADXL375 (high-g) acceleration into earth-frame velocity,
 * used only for the canard dynamic-pressure estimate (the LSM6 saturates at 16 g in boost).
 *
 * Major functions: DefaultConfig, Init, SetMode, AddMag, AddImu, AddHighG, CanardInput.
 * Pure C: no HAL, RTOS or allocation.
 */
#ifndef ATLAS_FLIGHT_ESTIMATOR_H
#define ATLAS_FLIGHT_ESTIMATOR_H
#include <stdbool.h>
#include <stdint.h>
#include "FusionAhrs.h"
#include "atlas_canard.h"
#include "atlas_sensor_samples.h"

/** @brief Which sensors feed Fusion. */
typedef enum
{
    ATLAS_FEST_CALIBRATE = 0, /**< Pad: gyro + accelerometer + magnetometer, bias learning. */
    ATLAS_FEST_GYRO_MAG = 1   /**< Flight: gyro + magnetometer; accelerometer ignored. */
} AtlasFlightEstimatorMode;

/** @brief Tuning and mounting. */
typedef struct
{
    float sample_rate_hz;       /**< Nominal IMU rate for Fusion's timeouts (480 Hz FIFO). */
    float gain;                 /**< Fusion gain (default 0.5). */
    float accel_rejection_deg;  /**< Fusion acceleration rejection (default 10 deg). */
    float mag_rejection_deg;    /**< Fusion magnetic rejection (default 20 deg). */
    float rejection_timeout_s;  /**< Fusion rejection timeout (default 5 s). */
    float gyro_range_dps;       /**< Gyro full scale for Fusion's overrange restart (4000). */
    bool use_magnetometer;      /**< Feed the MMC5983MA (default true). */
    float mag_to_imu[9];        /**< Row-major magnetometer-to-IMU axis map. UNVERIFIED default: identity. */
    float mag_offset_gauss[3];  /**< Hard-iron offset in magnetometer axes. UNVERIFIED default: zero. */
    float mag_min_gauss, mag_max_gauss; /**< Plausible field magnitude (0.2-0.8 G). */
    uint32_t mag_max_age_us;    /**< A magnetometer sample older than this is not fused (50 ms). */
    float bias_tau_s;           /**< Pad gyro-bias averaging time constant (default 10 s). */
    float bias_ready_s;         /**< Still time needed before the bias counts as calibrated (2 s). */
    float still_gyro_dps;       /**< "Still" means |gyro| below this (default 5 dps)... */
    float still_accel_g;        /**< ...and | |a| - 1 g | below this (default 0.05 g). */
    float max_dt_s;             /**< Longest interval integrated in one step (default 0.05 s). */
    float pad_velocity_tau_s;   /**< Pad velocity leak (default 1 s); not applied in flight. */
    float pad_tau_s;            /**< Pad averaging of "up" (LSM6) and the ADXL375 reading (default 5 s). */
    float adxl_max_dt_s;        /**< Longest ADXL375 interval integrated in one step (default 0.1 s). */
} AtlasFlightEstimatorConfig;

/** @brief Estimator state. */
typedef struct
{
    AtlasFlightEstimatorConfig config;
    FusionAhrs ahrs;
    AtlasFlightEstimatorMode mode;
    bool started;               /**< Fusion has an initial attitude. */
    bool have_time;
    uint32_t last_us;           /**< Timestamp of the last fused IMU sample. */
    float bias_dps[3];          /**< Gyro bias; frozen outside CALIBRATE. */
    float bias_still_s;         /**< Accumulated still time used for the bias. */
    bool still;                 /**< Latest sample looked still (|a| ~ 1 g, low rate). */
    float mag_imu[3];           /**< Latest conditioned magnetometer, IMU axes. */
    uint32_t mag_us;
    bool mag_valid;
    float accel_g[3];           /**< Latest accelerometer, g. */
    float gyro_dps[3];          /**< Latest bias-corrected gyro, dps. */
    float pad_up_imu[3];        /**< Mean "up" in IMU axes while still on the pad = rocket nose axis. */
    float adxl_pad_g[3];        /**< Mean ADXL375 reading while still on the pad (its own axes). */
    float pad_still_s;          /**< Still time behind the two pad averages. */
    float axial_velocity_m_s;   /**< Speed along the nose, single-integrated from the ADXL375. */
    float adxl_axial_g;         /**< Latest ADXL375 acceleration along the nose, offset removed. */
    float cos_tilt;             /**< Cosine of the nose's angle from vertical (1 on the pad). */
    uint32_t adxl_ms;           /**< Timestamp of the last ADXL375 sample integrated. */
    bool have_adxl;
    uint32_t samples, gaps, accel_saturated, gyro_saturated, mag_used;
} AtlasFlightEstimator;

/** @brief Fill defaults. @param c Result. */
void AtlasFlightEstimator_DefaultConfig(AtlasFlightEstimatorConfig *c);
/** @brief Reset to CALIBRATE with no attitude. @param e State. @param c Configuration (copied).
 * @return False on an invalid configuration. */
bool AtlasFlightEstimator_Init(AtlasFlightEstimator *e, const AtlasFlightEstimatorConfig *c);
/** @brief Switch sensor use. Leaving CALIBRATE freezes the gyro bias. @param e State. @param m Mode. */
void AtlasFlightEstimator_SetMode(AtlasFlightEstimator *e, AtlasFlightEstimatorMode m);
/** @brief Store a magnetometer sample for the next IMU steps. @param e State. @param m Sample. */
void AtlasFlightEstimator_AddMag(AtlasFlightEstimator *e, const AtlasMagSample *m);
/** @brief Fuse one IMU sample. @param e State. @param s Sample.
 * @return True if the sample advanced the attitude. */
bool AtlasFlightEstimator_AddImu(AtlasFlightEstimator *e, const AtlasImuSample *s);
/** @brief Single-integrate speed along the nose from one ADXL375 (high-g, +-200 g) sample, for
 * the canard dynamic-pressure estimate only. No ADXL375-to-IMU axis map is needed: on the pad
 * the rocket stands nose-up, so the ADXL375's own mean reading gives the nose direction in its
 * axes and its axial offset (|mean| - 1 g). In flight, gravity along the nose is cos(tilt) from
 * the Fusion attitude. It drifts with the remaining cross-axis offset and scale error.
 * @param e State. @param g Acceleration in ADXL375 axes (g). @param t_ms Sample tick.
 * @return True if integrated. */
bool AtlasFlightEstimator_AddHighG(AtlasFlightEstimator *e, const float g[3], uint32_t t_ms);
/** @brief Rocket nose direction in IMU axes, from "up" averaged while still on the pad.
 * @param e State. @param nose Result (unit vector). @return False before any pad still time. */
bool AtlasFlightEstimator_NoseImu(const AtlasFlightEstimator *e, float nose[3]);
/** @brief True once started, out of Fusion's startup ramp and with a calibrated bias. @param e State. */
bool AtlasFlightEstimator_Ready(const AtlasFlightEstimator *e);
/** @brief Speed along the nose (ADXL375 single integration), m/s. @param e State. */
float AtlasFlightEstimator_Speed(const AtlasFlightEstimator *e);
/** @brief Fill a canard input from the latest fused sample.
 * @param e State. @param airspeed_m_s Airspeed to use (negative = controller's own estimate).
 * @param enable Phase permission. @param parachute_deployed Recovery event. @param in Result.
 * @return False if there is no attitude yet. */
bool AtlasFlightEstimator_CanardInput(const AtlasFlightEstimator *e, float airspeed_m_s, bool enable,
                                      bool parachute_deployed, AtlasCanardInput *in);

#endif
