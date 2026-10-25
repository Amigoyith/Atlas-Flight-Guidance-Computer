/**
 * @file atlas_stabilization.h
 * @brief Pure gravity estimation and four radial-shaft bench servo geometry.
 * Major functions: Update fuses gyro/acceleration; Calibrate records upright;
 * Targets projects down into each horn plane; Seal/Valid protect SD settings.
 */
#ifndef ATLAS_STABILIZATION_H
#define ATLAS_STABILIZATION_H
#include <stdbool.h>
#include <stdint.h>
#include "FusionAhrs.h"

#define ATLAS_STAB_CHANNELS 4U
#define ATLAS_STAB_MASK 0xC3U
#define ATLAS_STAB_SAMPLE_MAX_MS 100U
#define ATLAS_STAB_LAUNCH_G 3.0f
#define ATLAS_STAB_LAUNCH_SAMPLES 3U
/** @brief Estimator flag bits, mirrored from the Fusion AHRS plus the launch latch. */
#define ATLAS_STAB_FLAG_LAUNCHED (1U<<0)           /**< Gyro-only; accelerometer locked out for good. */
#define ATLAS_STAB_FLAG_STARTUP (1U<<1)            /**< Fusion startup convergence ramp active. */
#define ATLAS_STAB_FLAG_ACCEL_IGNORED (1U<<2)      /**< Latest accelerometer sample not used. */
#define ATLAS_STAB_FLAG_ACCEL_RECOVERY (1U<<3)     /**< Rejection trigger passed 1.0; accelerometer forced back in. */
#define ATLAS_STAB_FLAG_OVERRANGE_RECOVERY (1U<<4) /**< Gyro overrange soft restart settling. */
/** @brief Build switch (CMake option ATLAS_STAB_USE_MAGNETOMETER, default OFF). Off, the magnetometer is
 * never given to Fusion: heading is not referenced to north (HEADING_VALID stays clear, MAG_IGNORED is set)
 * and the up vector is unaffected. Turn it on only after the constants below are verified. */
#ifndef ATLAS_STAB_USE_MAGNETOMETER
#define ATLAS_STAB_USE_MAGNETOMETER 0
#endif
/** @brief Magnetometer use. Pre-launch only, like the accelerometer: boost currents make it untrustworthy.
 * MAG_TO_IMU maps magnetometer axes into the IMU axes (row-major) and MAG_OFFSET_GAUSS is the hard-iron
 * offset in magnetometer axes. Both are UNVERIFIED defaults (identity, zero): the MMC5983MA/LSM6DSV16B
 * relative mounting is not documented, so confirm against the schematic and calibrate on the vehicle.
 * Heading is reported for the IMU axis HEADING_AXIS (2 = +Z, the body +X face normal), magnetic north. */
#define ATLAS_STAB_MAG_MAX_AGE_MS 250U
#define ATLAS_STAB_MAG_MIN_GAUSS 0.2f
#define ATLAS_STAB_MAG_MAX_GAUSS 0.8f
#define ATLAS_STAB_MAG_TO_IMU {1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f}
#define ATLAS_STAB_MAG_OFFSET_GAUSS {0.0f,0.0f,0.0f}
#define ATLAS_STAB_HEADING_AXIS 2U
#define ATLAS_STAB_HEADING_MIN_HORIZONTAL 0.2f
#define ATLAS_STAB_MAG_LOCK_DEG 5.0f
#define ATLAS_STAB_FLAG_MAG_IGNORED (1U<<5)        /**< Latest magnetometer sample absent or not used. */
#define ATLAS_STAB_FLAG_MAG_RECOVERY (1U<<6)       /**< Magnetic rejection trigger passed 1.0. */
#define ATLAS_STAB_FLAG_HEADING_VALID (1U<<7)      /**< Heading referenced to magnetic north and well defined. */
#define ATLAS_STAB_CONFIG_FILE "ASTAB.CFG"
#define ATLAS_STAB_CONFIG_BYTES 64U
#define ATLAS_STAB_COMMAND_PERIOD_MS 5U
#define ATLAS_STAB_COMMAND_WAKE_US 6U
#define ATLAS_STAB_COMMAND_QUIET_US 3U
#define ATLAS_STAB_COMMAND_QUIET_MS 150U

/** @brief Exact estimator/output check, retained at an active-control fault. */
typedef enum
{
    ATLAS_STAB_FAULT_NONE=0, ATLAS_STAB_FAULT_SAMPLE, ATLAS_STAB_FAULT_STALE,
    ATLAS_STAB_FAULT_RATE, ATLAS_STAB_FAULT_ACCEL, ATLAS_STAB_FAULT_VECTOR,
    ATLAS_STAB_FAULT_CORRECTION, ATLAS_STAB_FAULT_SETTLING,
    ATLAS_STAB_FAULT_GEOMETRY, ATLAS_STAB_FAULT_COMMAND, ATLAS_STAB_FAULT_NONFINITE
} AtlasStabilizationFault;

/** @brief Versioned little-endian IEEE-754 SD record, CRC32 over the first 60 bytes. */
typedef struct
{
    uint32_t magic, version, enabled, reverse_mask;
    float upright[3], bias_dps[3];
    uint32_t reserved[5], crc;
} AtlasStabilizationConfig;
/** @brief Latest new direct IMU sample, expressed in the sensor's right-handed axes. */
typedef struct
{
    float accel_g[3], gyro_dps[3];
    uint32_t timestamp_ms;
    bool valid;
} AtlasStabilizationSample;
/** @brief Optional latest magnetometer reading in its own axes (gauss); fed separately because it
 * arrives at a much lower rate than the IMU. */
typedef struct
{
    float gauss[3];
    uint32_t timestamp_ms;
    bool valid;
} AtlasStabilizationMagnetometer;
/** @brief Private estimator state; one owner, no HAL, allocation or time source.
 * Attitude is propagated by the Fusion AHRS; a zeroed state restarts it on the next sample. */
typedef struct
{
    FusionAhrs ahrs;
    float up[3], sum_accel[3], sum_gyro[3], previous_accel[3];
    uint32_t timestamp_ms, settled_ms, accel_ms, still_ms, still_count;
    AtlasStabilizationFault fault;
    bool initialized, ready, launched;
    uint32_t launch_count;
    uint32_t flags;           /* ATLAS_STAB_FLAG_*, refreshed on every AHRS step. */
    float accel_error_deg;    /* Angle between accelerometer and estimated gravity. */
    float accel_trigger;      /* Acceleration recovery trigger, 0.0-1.0; recovery at 1.0. */
    float mag_in[3];          /* Latest SetMagnetometer input, unconditioned. */
    uint32_t mag_in_ms;
    bool mag_in_valid;
    float mag_error_deg, mag_trigger; /* Magnetic rejection, as for the accelerometer. */
    float heading_deg;        /* Clockwise from magnetic north, 0-360, of ATLAS_STAB_HEADING_AXIS. */
    bool mag_locked;          /* Magnetometer has converged heading; retained while coasting after launch. */
    uint32_t mag_ms;          /* Tick of the last magnetometer sample fed to Fusion. */
} AtlasStabilization;
/** @brief Output diagnostics; channels are ordered PCB 7,8,1,2 throughout. */
typedef struct
{
    uint32_t state, reason, limited_mask, singular_mask, reverse_mask;
    AtlasStabilizationFault fault_detail;
    uint32_t ahrs_flags;      /* ATLAS_STAB_FLAG_* */
    float accel_error_deg, accel_trigger, mag_error_deg, mag_trigger, heading_deg;
    bool enabled, calibrated, calibration_ready, active, saved, save_busy;
    float up[3];
    uint16_t pulse_us[ATLAS_STAB_CHANNELS];
} AtlasStabilizationSnapshot;
/** @brief Per-shaft rest detection; moving shafts follow every new target directly. */
typedef struct
{
    uint32_t quiet_ms[ATLAS_STAB_CHANNELS];
    uint16_t anchor_us[ATLAS_STAB_CHANNELS];
    uint32_t tracking_mask;
} AtlasStabilizationCommands;

/** @brief Validate schema, CRC, finite calibration and fixed mounting. @param c Record.
 * @return True only for a complete supported record. */
bool AtlasStabilization_ConfigValid(const AtlasStabilizationConfig *c);
/** @brief Stamp schema and checksum after an explicit edit. @param c Record. */
void AtlasStabilization_Seal(AtlasStabilizationConfig *c);
/** @brief Consume new samples and expire retained data. @param s Owner state.
 * @param sample Latest sample. @param c Optional calibrated bias. @param now Current tick. */
void AtlasStabilization_Update(AtlasStabilization *s, const AtlasStabilizationSample *sample,
                               const AtlasStabilizationConfig *c, uint32_t now);
/** @brief Store the newest magnetometer reading; it is conditioned and used by the next Update.
 * Ignored after launch. Survives a restart of the estimator. @param s Estimator. @param m Reading or NULL to clear. */
void AtlasStabilization_SetMagnetometer(AtlasStabilization *s, const AtlasStabilizationMagnetometer *m);
/** @brief Latch launch: the accelerometer is never used again and attitude is gyro-only
 * from the current estimate. Cleared only by zeroing the state. Requires a started estimator.
 * @param s Estimator. @return False if there is no attitude yet to coast from. */
bool AtlasStabilization_Launch(AtlasStabilization *s);
/** @brief Capture one second of stationary upright samples; leaves mode disabled.
 * @param s Estimator. @param reverse Four direction bits. @param c Output record.
 * @return True if stationary, settled and USB-C-up mounting checks pass. */
bool AtlasStabilization_Calibrate(const AtlasStabilization *s, uint32_t reverse,
                                 AtlasStabilizationConfig *c);
/** @brief Project world-down into four independent radial-shaft horn planes.
 * @param c Valid calibration. @param up Estimated up in sensor axes.
 * @param previous Last commanded pulses, used at singularities. @param pulse Output targets.
 * @param limited Four bits for travel clipping. @param singular Four bits for axial gravity.
 * @return False on invalid inputs; no output may be asserted then. */
bool AtlasStabilization_Targets(const AtlasStabilizationConfig *c, const float up[3],
                                const uint16_t previous[4], uint16_t pulse[4],
                                uint32_t *limited, uint32_t *singular);
/** @brief Reset at the actual four-channel 1500 us start. @param s State. @param now Tick. */
void AtlasStabilization_CommandsReset(AtlasStabilizationCommands *s,uint32_t now);
/** @brief Follow direct targets every 5 ms while moving; hold small noise at rest.
 * Wake at 0.6 degrees. Rest after targets remain within 0.3 degrees for 150 ms.
 * @param s Rest state, committed only on success. @param target Geometric targets.
 * @param previous Actual preceding commanded pulses. @param singular Axial-gravity mask.
 * @param now Current tick.
 * @param pulse Output pulses, committed only on success. @return Input validity. */
bool AtlasStabilization_CommandsStep(AtlasStabilizationCommands *s,const uint16_t target[4],
                                     const uint16_t previous[4],uint32_t singular,uint32_t now,uint16_t pulse[4]);
#endif
