/**
 * @file atlas_flight_app.c
 * @brief Normal (flight) image application: phase state machine, state estimation,
 *        canard roll control and LoRa (RFD900x) telemetry in every phase.
 *
 * Every 10 ms, AtlasRtos_ApplicationStep():
 * 1. Fuses all new LSM6DSV16B FIFO samples (480 Hz, per-sample timestamps) and MMC5983MA
 *    samples (100 Hz) in the flight estimator (state estimation).
 * 2. Steps the flight state machine from the ADXL375 and MS5611 only (state transitions).
 *    ENGINE_ON switches the estimator to gyro + magnetometer.
 * 3. Steps the canard roll controller, enabled only in COAST; CHUTE neutralizes the
 *    canards and stops their PWM signal 20 s later.
 * 4. Drives the canard PWM channels (FLIGHT_OUTPUTS_QUALIFIED): only FLIGHT_ROLL_CANARD
 *    follows the controller; the other three, and all four whenever control is off or
 *    outputs are blocked, are held at neutral.
 * 5. Beeps the buzzer every 2 s once LANDED.
 * 6. Appends a snapshot to the SD card (FLTnnnn.BIN, one file per boot) every 100 ms.
 * 7. Sends a telemetry snapshot over the radio about once per second.
 *
 * Major functions: AtlasRtos_ApplicationStep() (overrides the weak hook).
 */
#include "atlas_build.h"
#if !ATLAS_BRINGUP
#include <math.h>
#include <string.h>
#include "atlas_buzzer.h"
#include "atlas_canard.h"
#include "atlas_flight.h"
#include "atlas_flight_estimator.h"
#include "atlas_io.h"
#include "atlas_rtos.h"
#include "atlas_stabilization.h"
#include "atlas_storage.h"
#include "atlas_telemetry.h"
#include "atlas_time.h"
#include "fatfs.h"
#include "stm32h7xx_hal.h"

/* ---------------------------------------------------------------------------------------
 * Vehicle configuration. Every value marked PLACEHOLDER must be replaced with measured or
 * structures-team values before flight; see docs/FLIGHT_STATE_MACHINE.md.
 * ------------------------------------------------------------------------------------- */

/** 1: the canard PWM channels are configured with flight_pwm[] and driven (test flight).
 * 0: the image computes and transmits everything but never configures or enables PWM.
 * The limits below are conservative defaults (+-10 deg around the KST 1500 us neutral);
 * check each linkage on the pad before launch. */
#define FLIGHT_OUTPUTS_QUALIFIED 1

#define FLIGHT_CANARDS 4U
/** Zero-based PWM channels driving the canards: PCB PWM 7, 8, 1, 2 (same as the bench). */
static const uint8_t flight_channel[FLIGHT_CANARDS] = {6U, 7U, 0U, 1U};
/** CHECK BEFORE LAUNCH: the one canard (index into flight_channel, 0 = PWM 7) that does roll
 * control. The other three are always held at their flight_pwm[] neutral (0 deg). */
#define FLIGHT_ROLL_CANARD 0U
/** CMake ATLAS_FLIGHT_ROLL_CONTROL (default ON). OFF: the roll canard is never moved, so all four
 * canards stay held straight; estimation, phases, logging and telemetry are unchanged. Use it
 * when the canard direction has not been checked. */
#ifndef ATLAS_FLIGHT_ROLL_CONTROL
#define ATLAS_FLIGHT_ROLL_CONTROL 1
#endif
/** Canard azimuths (body +X toward +Y). Roll-only coupled control gives every canard the same
 * deflection, so these only matter if pitch/yaw control is enabled later. */
static const float flight_azimuth_deg[FLIGHT_CANARDS] = {0.0f, 90.0f, 180.0f, 270.0f};
/** CHECK BEFORE LAUNCH: per-canard direction (+1/-1) so a canard pushes against a spin. Use the
 * ServoBench roll test: a channel with its Reverse bit set needs -1 here. Only the
 * FLIGHT_ROLL_CANARD entry moves a canard in flight. */
static const float flight_direction[FLIGHT_CANARDS] = {1.0f, 1.0f, 1.0f, 1.0f};
/** CHECK ON PAD: minimum/neutral/maximum pulse of each canard linkage (KST: 10 us/deg,
 * 1500 us = 0 deg). The canard controller itself stays within +-5 deg (1450-1550 us). */
static const AtlasPwmCalibration flight_pwm[FLIGHT_CANARDS] = {
    {1400U, 1500U, 1600U}, {1400U, 1500U, 1600U}, {1400U, 1500U, 1600U}, {1400U, 1500U, 1600U}};

#define FLIGHT_IMU_BATCH 32U
#define FLIGHT_RADIO_CHUNK 50U  /* 50 bytes at 57600 baud = 8.7 ms, under the 10 ms TX limit. */
#define FLIGHT_TM_PROFILE 2U    /* TM_PROFILE: 0 bringup, 1 ServoBench, 2 flight. */
#define FLIGHT_TM_VERSION 0x020200U
#define FLIGHT_BEEP_PERIOD_MS 2000U /* LANDED heartbeat: one beep every 2 s... */
#define FLIGHT_BEEP_MS 200U         /* ...200 ms long at the buzzer's resonant frequency. */
#define FLIGHT_LOG_PERIOD_MS 100U   /* SD log: one 512-byte snapshot every 100 ms (5 KB/s, 18 MB/h)... */
#define FLIGHT_LOG_RETRY_MS 1000U   /* ...an unmounted card is re-mounted at most this often... */
#define FLIGHT_LOG_MAX_FILE 9999U   /* ...and the last file name is appended once all are used. */

static bool flight_started;
static AtlasFlight flight;
static AtlasFlightEstimator estimator;
static AtlasCanard canard;
static AtlasCanardOutput canard_out;
static AtlasCanardConfig canard_config;
static bool canard_configured, canard_valid, canard_mounted, estimator_ready_at_launch;
static uint32_t canard_last_us;
static AtlasTelemetry telemetry;
static bool telemetry_ready;
static AtlasImuSample imu_batch[FLIGHT_IMU_BATCH];
static AtlasMagSample mag_batch[ATLAS_RTOS_MAG_RING_CAPACITY];
static AtlasIoSnapshot io_snapshot;
static AtlasRtosHealth rtos_health;
static uint32_t output_attempt_ms;
static bool output_config_sent, output_signal_off;
static uint32_t step_us, step_max_us; /* Duration of this hook (last, worst) vs its 10 ms period. */
static uint32_t beep_ms;
static uint32_t snapshot_words[ATLAS_TM_WORDS]; /* Shared by radio and SD snapshots (same task). */

/** SD flight log state (flight_log_step). */
typedef enum { FLIGHT_LOG_FIND = 0, FLIGHT_LOG_RUN } FlightLogState;
static struct
{
    FlightLogState state;
    uint16_t file;                      /* FLTnnnn.BIN number being probed, then written. */
    uint32_t outstanding;               /* Requests submitted and not yet completed. */
    uint32_t due_ms, mount_ms;
    uint32_t written, failed, dropped;  /* Records appended, append errors, records not submitted. */
} flight_log;
static AtlasStorageRequest log_request;
static AtlasStorageResult log_result;
static AtlasStorageHealth log_health;

/** @brief Log file name FLTnnnn.BIN. @param name Result (13 bytes). @param n File number. */
static void flight_log_name(char name[13], uint16_t n)
{
    memcpy(name, "FLT0000.BIN", 12U);
    for (unsigned i = 6U; i >= 3U; --i) { name[i] = (char)('0' + n % 10U); n /= 10U; }
}

/** Calibration word (telemetry TM_BNO_IO) bits; the ground waits for CAL_READY before launch. */
#define CAL_STARTED     (1U << 0) /* Fusion has an initial attitude. */
#define CAL_CONVERGED   (1U << 1) /* Fusion's 3 s startup ramp is over. */
#define CAL_BIAS_DONE   (1U << 2) /* Enough still time for the gyro bias. */
#define CAL_READY       (1U << 3) /* All of the above: ready to launch. */
#define CAL_STILL       (1U << 4) /* Rocket currently still (bias is being learned). */

/** @brief Canard controller for the flight: roll only, Apollyon-I-based guesses.
 * @param c Result. */
static void flight_canard_config(AtlasCanardConfig *c)
{
    AtlasCanard_RollOnlyConfig(c);
    c->canard_count = FLIGHT_CANARDS;
    for (unsigned i = 0U; i < FLIGHT_CANARDS; ++i)
    {
        c->canard[i].azimuth_deg = flight_azimuth_deg[i];
        c->canard[i].servo_center_us = (float)flight_pwm[i].neutral_us;
        c->canard[i].servo_us_per_deg = 10.0f * flight_direction[i]; /* KST X10: 10 us/deg. */
    }
    /* GUESS: 27.5 cm^2 each. The controller assumes all four canards deflect together; only
     * FLIGHT_ROLL_CANARD does, so a quarter of the area makes it command the deflection one
     * canard needs for the same roll moment (still capped at max_deflection_deg). */
    c->canard_area_m2 = 0.00275f / (float)FLIGHT_CANARDS;
    c->canard_lift_slope_per_rad = 2.0f;     /* GUESS: replace with CFD. */
    c->canard_arm_m = 0.95f;                 /* GUESS. */
    c->canard_radius_m = 0.10f;              /* GUESS. */
    c->inertia_kg_m2[0] = c->inertia_kg_m2[1] = 14.0f; /* ESTIMATE: Apollyon I at burnout. */
    c->inertia_kg_m2[2] = 0.08f;             /* ESTIMATE. */
    c->air_density_kg_m3 = 1.0f;
    c->max_deflection_deg = 5.0f;            /* First active flight. */
    c->min_pulse_us = 1000.0f;               /* Final clamp is the flight_pwm[] range. */
    c->max_pulse_us = 2000.0f;
    /* sensor_from_body is set at launch from the nose axis measured on the pad
     * (flight_mount_canards), so the board may be mounted in any orientation. */
}

/** @brief Point the canard controller's roll axis along the nose measured on the pad.
 * Azimuth 0 is taken from the IMU axis most perpendicular to the nose; for roll-only control
 * with coupled canards it does not matter. @return True if mounted. */
static bool flight_mount_canards(void)
{
    float nose[3];
    if (!AtlasFlightEstimator_NoseImu(&estimator, nose)) return false;
    float hint[3] = {0.0f, 0.0f, 0.0f};
    unsigned k = 0U;
    for (unsigned i = 1U; i < 3U; ++i) if (fabsf(nose[i]) < fabsf(nose[k])) k = i;
    hint[k] = 1.0f;
    if (!AtlasCanard_MountFromAxes(nose, hint, canard_config.sensor_from_body)) return false;
    canard_configured = AtlasCanard_Init(&canard, &canard_config);
    return canard_configured;
}

/** @brief One-time setup on the first hook call. @param now_ms Tick. */
static void flight_start(uint32_t now_ms)
{
    AtlasFlightConfig fc;
    AtlasFlight_DefaultConfig(&fc);
    (void)AtlasFlight_Init(&flight, &fc, now_ms);
    AtlasFlightEstimatorConfig ec;
    AtlasFlightEstimator_DefaultConfig(&ec);
    /* Same magnetometer setting as the stabilization estimator: CMake ATLAS_STAB_USE_MAGNETOMETER
     * (default OFF) and its unverified axis map/offset. Off, roll is gyro-only after launch. */
    static const float mag_map[9] = ATLAS_STAB_MAG_TO_IMU, mag_offset[3] = ATLAS_STAB_MAG_OFFSET_GAUSS;
    ec.use_magnetometer = ATLAS_STAB_USE_MAGNETOMETER != 0;
    memcpy(ec.mag_to_imu, mag_map, sizeof(mag_map));
    memcpy(ec.mag_offset_gauss, mag_offset, sizeof(mag_offset));
    (void)AtlasFlightEstimator_Init(&estimator, &ec);
    flight_canard_config(&canard_config);
    canard_configured = AtlasCanard_Init(&canard, &canard_config);
    const uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    uint32_t boot[2];
    AtlasTelemetry_NewBoot(boot);
    AtlasTelemetry_Init(&telemetry, uid, boot, now_ms);
    AtlasTelemetry_SetStreaming(&telemetry, true);
    telemetry_ready = true;
    flight_started = true;
}

/** @brief Fuse every new magnetometer and IMU sample (state estimation). */
static void flight_estimate(void)
{
    const size_t mags = AtlasRtos_ReadMagSamples(mag_batch, ATLAS_RTOS_MAG_RING_CAPACITY);
    for (size_t i = 0U; i < mags; ++i) AtlasFlightEstimator_AddMag(&estimator, &mag_batch[i]);
    size_t n;
    do
    {
        n = AtlasRtos_ReadImuSamples(imu_batch, FLIGHT_IMU_BATCH);
        for (size_t i = 0U; i < n; ++i) (void)AtlasFlightEstimator_AddImu(&estimator, &imu_batch[i]);
    } while (n == FLIGHT_IMU_BATCH);
}

/** @brief Step the phase machine from the ADXL375 and barometer (state transitions).
 * @param s Snapshot. @param now_ms Tick. */
static void flight_transitions(const AtlasRtosSnapshot *s, uint32_t now_ms)
{
    const AtlasFlightPhase before = flight.phase;
    AtlasFlightInput in = {0};
    in.adxl_g[0] = s->adxl375.x_g; in.adxl_g[1] = s->adxl375.y_g; in.adxl_g[2] = s->adxl375.z_g;
    in.adxl_ms = s->adxl375.timestamp_ms;
    in.adxl_valid = (s->valid_mask & ATLAS_RTOS_VALID_ADXL375) != 0U && s->adxl375_status == ATLAS_OK;
    in.pressure_pa = s->ms5611.pressure_pa;
    in.baro_ms = s->ms5611.timestamp_ms;
    in.baro_valid = (s->valid_mask & ATLAS_RTOS_VALID_MS5611) != 0U;
    in.now_ms = now_ms;
    const AtlasFlightPhase after = AtlasFlight_Step(&flight, &in);
    /* The same ADXL375 sample also feeds the canard speed estimate (state estimation). */
    if (in.adxl_valid) (void)AtlasFlightEstimator_AddHighG(&estimator, in.adxl_g, in.adxl_ms);
    if (before == ATLAS_FLIGHT_PAD && after != ATLAS_FLIGHT_PAD)
    {
        /* Launch: the accelerometer saturates in boost and reads drag in coast, so attitude
         * continues on gyro + magnetometer only. Bias is frozen from the pad calibration. */
        estimator_ready_at_launch = AtlasFlightEstimator_Ready(&estimator);
        AtlasFlightEstimator_SetMode(&estimator, ATLAS_FEST_GYRO_MAG);
        canard_mounted = flight_mount_canards();
    }
}

/** @brief Run the canard controller on the newest fused sample. */
static void flight_control(void)
{
    if (!canard_configured || estimator.last_us == canard_last_us) return;
    const bool enable = ATLAS_FLIGHT_ROLL_CONTROL && flight.phase == ATLAS_FLIGHT_COAST &&
                        estimator_ready_at_launch && canard_mounted;
    AtlasCanardInput in;
    if (!AtlasFlightEstimator_CanardInput(&estimator, AtlasFlightEstimator_Speed(&estimator), enable,
                                          flight.phase >= ATLAS_FLIGHT_CHUTE, &in))
    {
        canard_valid = false;
        return;
    }
    canard_last_us = estimator.last_us;
    (void)AtlasCanard_Step(&canard, &in, &canard_out);
    canard_valid = canard_out.state != ATLAS_CANARD_INVALID_INPUT && canard_out.state != ATLAS_CANARD_NOT_CONFIGURED;
}

/** @brief Configure, enable, update and finally disable the canard PWM channels.
 * @param now_ms Tick. */
static void flight_outputs(uint32_t now_ms)
{
    AtlasIoResult result;
    while (AtlasIo_Receive(&result)) {} /* Sole consumer; results are observed via the snapshot. */
    if (!FLIGHT_OUTPUTS_QUALIFIED || output_signal_off) return;
    uint8_t mask = 0U;
    for (unsigned i = 0U; i < FLIGHT_CANARDS; ++i) mask |= (uint8_t)(1U << flight_channel[i]);
    AtlasIoCommand cmd;
    memset(&cmd, 0, sizeof(cmd));

    if (canard_out.signal_off)
    {
        cmd.type = ATLAS_IO_PWM_DISABLE;
        cmd.arguments.channel_mask = mask;
        if (AtlasIo_Submit(&cmd, NULL) == ATLAS_OK) output_signal_off = true;
        return;
    }
    if (!AtlasIo_GetSnapshot(&io_snapshot) || !AtlasRtos_OutputsPermitted()) return;
    if (!io_snapshot.configured)
    {
        if (output_config_sent && (uint32_t)(now_ms - output_attempt_ms) < 1000U) return;
        cmd.type = ATLAS_IO_CONFIGURE;
        cmd.arguments.configuration.electrical_review_complete = true;
        cmd.arguments.configuration.pwm_allowed_mask = mask;
        for (unsigned i = 0U; i < FLIGHT_CANARDS; ++i)
            cmd.arguments.configuration.pwm[flight_channel[i]] = flight_pwm[i];
        output_config_sent = AtlasIo_Submit(&cmd, NULL) == ATLAS_OK;
        output_attempt_ms = now_ms;
        return;
    }
    if ((io_snapshot.pwm_enabled_mask & mask) != mask)
    {
        /* Enable at neutral on the pad so the canards are held through boost; retried after
         * any permission loss (which stops PWM). */
        if ((uint32_t)(now_ms - output_attempt_ms) < 1000U) return;
        cmd.type = ATLAS_IO_PWM_ENABLE;
        cmd.arguments.channel_mask = mask;
        (void)AtlasIo_Submit(&cmd, NULL);
        output_attempt_ms = now_ms;
        return;
    }
    cmd.type = ATLAS_IO_PWM_SET_MULTI;
    cmd.arguments.pwm_multi.channel_mask = mask;
    for (unsigned i = 0U; i < FLIGHT_CANARDS; ++i)
    {
        const AtlasPwmCalibration *p = &flight_pwm[i];
        uint16_t us = (ATLAS_FLIGHT_ROLL_CONTROL && canard_valid && i == FLIGHT_ROLL_CANARD) ?
            canard_out.pulse_us[i] : p->neutral_us;
        if (us < p->minimum_us) us = p->minimum_us;
        if (us > p->maximum_us) us = p->maximum_us;
        cmd.arguments.pwm_multi.pulse_us[flight_channel[i]] = us;
    }
    (void)AtlasIo_Submit(&cmd, NULL);
}

/** @brief Radio send callback: split one frame into bounded RFD900x writes.
 * @param context Unused. @param bytes Frame. @param length Bytes. @return Queue status. */
static AtlasStatus flight_radio_send(void *context, const uint8_t *bytes, size_t length)
{
    (void)context;
    AtlasStatus status = ATLAS_OK;
    for (size_t at = 0U; at < length && status == ATLAS_OK; at += FLIGHT_RADIO_CHUNK)
    {
        AtlasRtosCommandRequest r;
        memset(&r, 0, sizeof(r));
        r.type = ATLAS_RTOS_COMMAND_RADIO_WRITE;
        r.arguments.stream.length = (uint16_t)((length - at) < FLIGHT_RADIO_CHUNK ? (length - at) : FLIGHT_RADIO_CHUNK);
        memcpy(r.arguments.stream.bytes, bytes + at, r.arguments.stream.length);
        r.arguments.stream.timeout_ms = ATLAS_RTOS_MAX_STREAM_TX_TIMEOUT_MS;
        status = AtlasRtos_SubmitCommand(&r, 0U, NULL);
    }
    return status;
}

/** @brief Encode a scaled value; INT32_MIN marks unavailable. @param v Value. @param scale Multiplier.
 * @return Signed bits. */
static uint32_t flight_tm(float v, float scale)
{
    const float n = roundf(v * scale);
    return (isfinite(n) && n > -2147483520.0f && n < 2147483520.0f) ? (uint32_t)(int32_t)n : UINT32_C(0x80000000);
}

/** @brief Build one snapshot (radio telemetry and SD log share it). Sensor words keep the bench
 * layout; the BNO085 words (unused, BNO disabled) carry the flight state. The CRC word is left 0.
 * @param s Snapshot. @param now_ms Tick. @param w Result, ATLAS_TM_WORDS words. */
static void flight_snapshot(const AtlasRtosSnapshot *s, uint32_t now_ms, uint32_t *w)
{
    memset(w, 0, ATLAS_TM_WORDS * sizeof(w[0]));
    w[TM_MAGIC] = ATLAS_TM_MAGIC; w[TM_UPTIME] = now_ms; w[TM_VERSION] = FLIGHT_TM_VERSION;
    w[TM_PROFILE] = FLIGHT_TM_PROFILE;
    w[TM_SAMPLE_STATUS] = s->adxl375_status; w[TM_SAMPLE_STATUS + 1] = s->lsm6dsv16b_status;
    w[TM_SAMPLE_STATUS + 2] = s->mmc5983ma_status; w[TM_SAMPLE_STATUS + 3] = s->ms5611_status;
    w[TM_TIME] = s->adxl375.timestamp_ms; w[TM_TIME + 1] = s->lsm6dsv16b.timestamp_ms;
    w[TM_TIME + 2] = s->mmc5983ma.timestamp_ms; w[TM_TIME + 3] = s->ms5611.timestamp_ms;
    w[TM_ADXL] = flight_tm(s->adxl375.x_g, 1000.0f); w[TM_ADXL + 1] = flight_tm(s->adxl375.y_g, 1000.0f);
    w[TM_ADXL + 2] = flight_tm(s->adxl375.z_g, 1000.0f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        w[TM_LSM_ACCEL + i] = flight_tm(estimator.accel_g[i], 1000.0f);
        w[TM_LSM_GYRO + i] = flight_tm(estimator.gyro_dps[i], 1000.0f);
    }
    w[TM_MMC] = flight_tm(s->mmc5983ma.x_gauss, 100000.0f); w[TM_MMC + 1] = flight_tm(s->mmc5983ma.y_gauss, 100000.0f);
    w[TM_MMC + 2] = flight_tm(s->mmc5983ma.z_gauss, 100000.0f);
    w[TM_BARO_PA] = (uint32_t)s->ms5611.pressure_pa; w[TM_BARO_TEMP] = (uint32_t)s->ms5611.temperature_centi_c;
    /* Flight state in the BNO085 words. */
    w[TM_BNO_COUNT] = (uint32_t)flight.phase; w[TM_BNO_COUNT + 1] = (uint32_t)flight.reason;
    w[TM_BNO_COUNT + 2] = now_ms - flight.phase_since_ms; w[TM_BNO_COUNT + 3] = flight.launch_ms;
    w[TM_BNO_TIME] = flight_tm(flight.altitude_m, 100.0f); w[TM_BNO_TIME + 1] = flight_tm(flight.climb_m_s, 100.0f);
    w[TM_BNO_TIME + 2] = flight_tm(flight.max_altitude_m, 100.0f); w[TM_BNO_TIME + 3] = flight_tm(flight.accel_g, 1000.0f);
    w[TM_BNO_ACCURACY] = (uint32_t)estimator.started | ((uint32_t)AtlasFlightEstimator_Ready(&estimator) << 1U) |
        ((uint32_t)estimator.mode << 2U) | ((uint32_t)estimator_ready_at_launch << 3U) |
        ((uint32_t)canard_out.state << 8U) | ((uint32_t)canard_out.signal_off << 16U) |
        ((uint32_t)canard_out.roll_holding << 17U) | ((uint32_t)canard_out.saturated << 18U) |
        ((uint32_t)FLIGHT_OUTPUTS_QUALIFIED << 19U) | ((uint32_t)AtlasRtos_OutputsPermitted() << 20U);
    w[TM_BNO_ACCEL] = flight_tm(estimator.axial_velocity_m_s, 100.0f);
    w[TM_BNO_ACCEL + 1] = flight_tm(estimator.adxl_axial_g, 1000.0f);
    w[TM_BNO_ACCEL + 2] = flight_tm(estimator.cos_tilt, 1000.0f);
    for (unsigned i = 0U; i < 3U; ++i) w[TM_BNO_GYRO + i] = flight_tm(canard_out.body_rate_dps[i], 1000.0f);
    w[TM_BNO_MAG] = flight_tm(canard_out.deflection_deg[0], 1000.0f);
    w[TM_BNO_MAG + 1] = flight_tm(canard_out.authority, 1000.0f);
    w[TM_BNO_MAG + 2] = flight_tm(canard_out.dynamic_pressure_pa, 1.0f);
    const FusionQuaternion q = FusionAhrsGetQuaternion(&estimator.ahrs);
    for (unsigned i = 0U; i < 4U; ++i) w[TM_BNO_Q + i] = flight_tm(estimator.started ? q.array[i] : NAN, 1000000.0f);
    const AtlasGnssNavPvt *g = &s->gnss_nav_pvt;
    w[TM_GNSS_TIME] = g->received_at_ms;
    w[TM_GNSS_FIX] = g->fix_type | ((uint32_t)g->flags << 8U) | ((uint32_t)g->satellites_used << 16U);
    w[TM_LAT] = (uint32_t)g->latitude_1e7_deg; w[TM_LON] = (uint32_t)g->longitude_1e7_deg;
    w[TM_ALT] = (uint32_t)g->height_msl_mm; w[TM_HACC] = g->horizontal_accuracy_mm; w[TM_TOW] = g->time_of_week_ms;
    /* Calibration status for the ground: launch only once CAL_READY is set. */
    const AtlasFlightEstimatorConfig *ec = &estimator.config;
    float progress = estimator.bias_still_s / ec->bias_ready_s;
    if (progress > 1.0f) progress = 1.0f;
    uint32_t cal = (estimator.started ? CAL_STARTED : 0U) |
        ((estimator.started && !FusionAhrsGetFlags(&estimator.ahrs).startup) ? CAL_CONVERGED : 0U) |
        (estimator.bias_still_s >= ec->bias_ready_s ? CAL_BIAS_DONE : 0U) |
        (AtlasFlightEstimator_Ready(&estimator) ? CAL_READY : 0U) | (estimator.still ? CAL_STILL : 0U);
    cal |= (uint32_t)(progress * 100.0f) << 8U;
    w[TM_BNO_IO] = cal;
    w[TM_BNO_PROTOCOL] = (uint32_t)(estimator.bias_still_s * 1000.0f);
    /* Supervision: why outputs are blocked, error counters and the hook's timing margin. */
    if (AtlasRtos_GetHealth(&rtos_health, 0U))
    {
        w[TM_FAULT] = (uint32_t)rtos_health.fault | ((uint32_t)rtos_health.output_block << 8U);
        w[TM_ERRORS] = rtos_health.sampling_errors; w[TM_ERRORS + 1] = rtos_health.service_errors;
        w[TM_ERRORS + 2] = rtos_health.io_deadline_misses; w[TM_ERRORS + 3] = rtos_health.application_deadline_misses;
    }
    w[TM_COUNT] = step_us; w[TM_COUNT + 1] = step_max_us;
    w[TM_COUNT + 2] = estimator.samples; w[TM_COUNT + 3] = estimator.mag_used;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        w[TM_INIT + i] = flight_tm(estimator.adxl_pad_g[i], 1000.0f);     /* ADXL375 pad mean (mg) */
        w[TM_INIT + 3U + i] = flight_tm(estimator.pad_up_imu[i], 1000.0f); /* nose in IMU axes */
    }
    w[TM_INIT + 6] = estimator.accel_saturated;
    w[TM_INIT + 7] = (uint32_t)(estimator.pad_still_s * 1000.0f);
    w[TM_INIT + 8] = canard_mounted ? 1U : 0U;
    w[TM_INIT + 9] = flight.apogee_ms;
    w[TM_INIT + 10] = FLIGHT_ROLL_CANARD;
    w[TM_STABILIZATION] = estimator.gaps;
    for (unsigned i = 0U; i < 3U; ++i) w[TM_UP + i] = flight_tm(estimator.bias_dps[i], 1000.0f);
    for (unsigned i = 0U; i < 4U; ++i)
        w[TM_PWM + i] = io_snapshot.commanded_pwm_us[2U * i] | ((uint32_t)io_snapshot.commanded_pwm_us[2U * i + 1U] << 16U);
    w[TM_GPIO] = (uint32_t)io_snapshot.pwm_enabled_mask << 17U;
    w[TM_TX_PACKETS] = telemetry.view.stats.tx_packets;
    /* SD log status: card/mounted/logging bits and file number; records written, failed, dropped. */
    w[TM_SD] = (uint32_t)log_health.card_detected | ((uint32_t)log_health.mounted << 1U) |
        ((uint32_t)(flight_log.state == FLIGHT_LOG_RUN) << 2U) | ((uint32_t)flight_log.file << 16U);
    w[TM_SD_ERRORS] = flight_log.written; w[TM_SERVICE] = flight_log.failed; w[TM_STARTUP] = flight_log.dropped;
}

/** @brief Start the next radio telemetry snapshot. @param s Snapshot. @param now_ms Tick. */
static void flight_telemetry_begin(const AtlasRtosSnapshot *s, uint32_t now_ms)
{
    flight_snapshot(s, now_ms, snapshot_words);
    (void)AtlasTelemetry_Begin(&telemetry, snapshot_words, now_ms);
}

/** @brief Submit a mount request when the card is present but not mounted (at most once per
 * FLIGHT_LOG_RETRY_MS). @param now_ms Tick. */
static void flight_log_mount(uint32_t now_ms)
{
    if (!log_health.card_detected || (uint32_t)(now_ms - flight_log.mount_ms) < FLIGHT_LOG_RETRY_MS) return;
    memset(&log_request, 0, sizeof(log_request));
    log_request.operation = ATLAS_STORAGE_MOUNT;
    if (AtlasStorage_Submit(&log_request, NULL) == ATLAS_OK) ++flight_log.outstanding;
    flight_log.mount_ms = now_ms;
}

/** @brief SD flight log: find the first unused FLTnnnn.BIN, then append one CRC-checked 512-byte
 * snapshot (the radio format) every FLIGHT_LOG_PERIOD_MS. Never waits: a missing, full or slow
 * card only drops records, and the storage owner runs below every flight task.
 * @param s Snapshot. @param now_ms Tick. */
static void flight_log_step(const AtlasRtosSnapshot *s, uint32_t now_ms)
{
    (void)AtlasStorage_GetHealth(&log_health);
    while (AtlasStorage_Receive(&log_result)) /* Sole consumer in the flight image. */
    {
        if (flight_log.outstanding != 0U) --flight_log.outstanding;
        if (log_result.operation == ATLAS_STORAGE_APPEND)
        {
            if (log_result.status == ATLAS_OK) ++flight_log.written; else ++flight_log.failed;
        }
        else if (log_result.operation == ATLAS_STORAGE_READ && flight_log.state == FLIGHT_LOG_FIND)
        {
            if (log_result.status == ATLAS_OK && flight_log.file < FLIGHT_LOG_MAX_FILE) ++flight_log.file;
            else if (log_result.status == ATLAS_OK || log_result.filesystem_result == (uint8_t)FR_NO_FILE)
                flight_log.state = FLIGHT_LOG_RUN; /* Unused name found (or the last one, appended). */
        }
    }
    if (!log_health.mounted) flight_log_mount(now_ms);
    if (flight_log.state == FLIGHT_LOG_FIND)
    {
        /* One probe at a time: a READ of FLTnnnn.BIN that fails with "no file" picks that name. */
        if (flight_log.outstanding != 0U || !log_health.mounted) return;
        memset(&log_request, 0, sizeof(log_request));
        log_request.operation = ATLAS_STORAGE_READ;
        flight_log_name(log_request.filename, flight_log.file);
        log_request.length = 1U;
        if (AtlasStorage_Submit(&log_request, NULL) == ATLAS_OK) ++flight_log.outstanding;
        return;
    }
    if ((uint32_t)(now_ms - flight_log.due_ms) < FLIGHT_LOG_PERIOD_MS) return;
    flight_log.due_ms = now_ms;
    if (!log_health.mounted) { ++flight_log.dropped; return; }
    flight_snapshot(s, now_ms, snapshot_words);
    memset(&log_request, 0, sizeof(log_request));
    log_request.operation = ATLAS_STORAGE_APPEND;
    flight_log_name(log_request.filename, flight_log.file);
    log_request.length = ATLAS_STORAGE_DATA_CAPACITY;
    memcpy(log_request.data, snapshot_words, 4U * TM_CRC); /* Little-endian words, as on the radio. */
    const uint32_t crc = AtlasTelemetry_Crc(log_request.data, 4U * TM_CRC);
    memcpy(log_request.data + 4U * TM_CRC, &crc, sizeof(crc));
    if (AtlasStorage_Submit(&log_request, NULL) == ATLAS_OK) ++flight_log.outstanding; else ++flight_log.dropped;
}

/** @brief Drain the radio and send the next telemetry packet when due.
 * @param s Snapshot. @param now_ms Tick. */
static void flight_telemetry(const AtlasRtosSnapshot *s, uint32_t now_ms)
{
    uint8_t rx[64];
    size_t n;
    while ((n = AtlasRtos_ReadRadio(rx, sizeof(rx), 0U)) != 0U) AtlasTelemetry_Feed(&telemetry, rx, n, now_ms);
    if (s->radio_command_mode) return;
    if (AtlasTelemetry_Due(&telemetry, now_ms)) flight_telemetry_begin(s, now_ms);
    AtlasTelemetry_Service(&telemetry, now_ms, flight_radio_send, NULL);
}

/** @brief LANDED heartbeat for recovery: one beep every FLIGHT_BEEP_PERIOD_MS. A full command
 * queue just retries on the next step. @param now_ms Tick. */
static void flight_buzzer(uint32_t now_ms)
{
    if (flight.phase != ATLAS_FLIGHT_LANDED || (uint32_t)(now_ms - beep_ms) < FLIGHT_BEEP_PERIOD_MS) return;
    AtlasRtosCommandRequest r;
    memset(&r, 0, sizeof(r));
    r.type = ATLAS_RTOS_COMMAND_BUZZER_BEEP;
    r.arguments.buzzer.frequency_hz = ATLAS_BUZZER_RESONANT_FREQUENCY_HZ;
    r.arguments.buzzer.duration_ms = FLIGHT_BEEP_MS;
    if (AtlasRtos_SubmitCommand(&r, 0U, NULL) == ATLAS_OK) beep_ms = now_ms;
}

/** @brief 100 Hz flight application. @param snapshot Coherent sensor snapshot. @param now_ms Tick. */
void AtlasRtos_ApplicationStep(const AtlasRtosSnapshot *snapshot, uint32_t now_ms)
{
    const uint32_t start_us = AtlasTime_NowUs();
    if (!flight_started) flight_start(now_ms);
    flight_estimate();
    flight_transitions(snapshot, now_ms);
    flight_control();
    flight_outputs(now_ms);
    flight_buzzer(now_ms);
    flight_log_step(snapshot, now_ms);
    if (telemetry_ready) flight_telemetry(snapshot, now_ms);
    step_us = AtlasTime_NowUs() - start_us;
    if (step_us > step_max_us) step_max_us = step_us;
}
#endif
