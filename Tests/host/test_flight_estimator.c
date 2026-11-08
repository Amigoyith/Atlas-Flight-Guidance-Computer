/**
 * @file test_flight_estimator.c
 * @brief Host tests for the flight estimator: sample timing, pad calibration, accelerometer
 *        lockout in flight, magnetometer heading hold and velocity integration.
 * Major functions: main runs each case on synthetic LSM6DSV16B / MMC5983MA streams.
 */
#include "atlas_flight_estimator.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, msg); ++failures; } } while (0)
#define G 9.80665f
#define RAD (3.14159265358979f / 180.0f)

/** @brief Build one IMU sample. */
static AtlasImuSample imu(uint32_t t_us, const float accel_g[3], const float gyro_dps[3])
{
    AtlasImuSample s;
    memset(&s, 0, sizeof(s));
    s.timestamp_us = t_us;
    for (unsigned i = 0; i < 3U; ++i) { s.accel_m_s2[i] = accel_g[i] * G; s.gyro_rad_s[i] = gyro_dps[i] * RAD; }
    s.flags = ATLAS_IMU_FLAG_GYRO_VALID | ATLAS_IMU_FLAG_ACCEL_VALID;
    return s;
}

/** @brief Heading of sensor +X in ENU, degrees from east toward north. */
static float yaw_deg(const AtlasFlightEstimator *e)
{
    const FusionMatrix r = FusionQuaternionToMatrix(FusionQuaternionNormalise(FusionAhrsGetQuaternion(&e->ahrs)));
    return atan2f(r.element.yx, r.element.xx) / RAD;
}

/** @brief Angle between sensor +Z and ENU up, degrees. */
static float tilt_deg(const AtlasFlightEstimator *e)
{
    const FusionMatrix r = FusionQuaternionToMatrix(FusionQuaternionNormalise(FusionAhrsGetQuaternion(&e->ahrs)));
    return acosf(fminf(1.0f, r.element.zz)) / RAD;
}

/** @brief Fresh estimator with the magnetometer off unless requested. */
static void make(AtlasFlightEstimator *e, bool mag)
{
    AtlasFlightEstimatorConfig c;
    AtlasFlightEstimator_DefaultConfig(&c);
    c.use_magnetometer = mag;
    CHECK(AtlasFlightEstimator_Init(e, &c), "init");
}

/** @brief Sit still upright for a time at 480 Hz with a gyro bias. @return Next timestamp. */
static uint32_t pad(AtlasFlightEstimator *e, uint32_t t, float seconds, const float bias[3])
{
    const float up[3] = {0.0f, 0.0f, 1.0f};
    const uint32_t n = (uint32_t)(seconds * 480.0f);
    for (uint32_t i = 0; i < n; ++i) { AtlasImuSample s = imu(t, up, bias); AtlasFlightEstimator_AddImu(e, &s); t += 2083U; }
    return t;
}

/** @brief Timing: a 90 dps yaw for exactly 1 s must integrate to 90 deg at the nominal 480 Hz
 * and with uneven sample spacing. Fusion uses each sample's own timestamp, so the 100 Hz
 * application cadence and the FIFO batch sizes cannot change the result. */
static void timing_case(void)
{
    const float zero[3] = {0}, up[3] = {0.0f, 0.0f, 1.0f}, rate[3] = {0.0f, 0.0f, 90.0f};
    float results[2];
    for (int mode = 0; mode < 2; ++mode)
    {
        AtlasFlightEstimator e;
        make(&e, false);
        uint32_t t = pad(&e, 1000000U, 3.0f, zero);
        AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
        const float y0 = yaw_deg(&e);
        /* mode 0: exact 480 Hz; mode 1: +-20 % spacing jitter over the same total time. */
        const uint32_t t_end = t + 1000000U;
        uint32_t k = 0U;
        while (t < t_end)
        {
            uint32_t step = 2083U;
            if (mode == 1) step = (k % 3U == 0U) ? 2500U : ((k % 3U == 1U) ? 1666U : 2083U);
            if (t + step > t_end) step = t_end - t;
            t += step;
            AtlasImuSample s = imu(t, up, rate);
            AtlasFlightEstimator_AddImu(&e, &s);
            ++k;
        }
        float d = yaw_deg(&e) - y0;
        if (d < -180.0f) d += 360.0f;
        results[mode] = d;
    }
    printf("  90 dps for 1 s -> %.3f deg (480 Hz) / %.3f deg (jittered spacing)\n", results[0], results[1]);
    for (int m = 0; m < 2; ++m) CHECK(fabsf(results[m] - 90.0f) < 0.2f, "yaw integrates with per-sample timing");

    /* For comparison, millisecond timestamps (the bench estimator's resolution): each 2.083 ms
     * interval reads as 2 or 3 ms, but the errors telescope, so the total stays close. */
    AtlasFlightEstimator e;
    make(&e, false);
    uint32_t t = pad(&e, 1000000U, 3.0f, zero);
    AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
    const float y0 = yaw_deg(&e);
    for (uint32_t i = 1U; i <= 480U; ++i)
    {
        const uint32_t ms_stamp = (t + i * 2083U) / 1000U * 1000U; /* ms-quantized */
        AtlasImuSample s = imu(ms_stamp, up, rate);
        AtlasFlightEstimator_AddImu(&e, &s);
    }
    printf("  same motion with ms-quantized stamps -> %.3f deg\n", yaw_deg(&e) - y0);
}

/** @brief Pad calibration learns the gyro bias, after which the estimator is Ready. */
static void bias_case(void)
{
    AtlasFlightEstimator e;
    make(&e, false);
    const float bias[3] = {1.5f, -2.0f, 0.7f};
    uint32_t t = pad(&e, 1000000U, 1.0f, bias);
    CHECK(!AtlasFlightEstimator_Ready(&e), "not ready after 1 s");
    t = pad(&e, t, 20.0f, bias);
    CHECK(AtlasFlightEstimator_Ready(&e), "ready after the pad calibration");
    for (unsigned i = 0; i < 3U; ++i) CHECK(fabsf(e.bias_dps[i] - bias[i]) < 0.05f, "bias learned");
    /* With the bias removed, 10 s of still gyro-only flight drifts < 1 deg in yaw. */
    AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
    const float y0 = yaw_deg(&e);
    t = pad(&e, t, 10.0f, bias);
    CHECK(fabsf(yaw_deg(&e) - y0) < 1.0f, "bias-corrected drift small");
    printf("  bias %.3f %.3f %.3f dps, 10 s gyro-only yaw drift %.3f deg\n",
           e.bias_dps[0], e.bias_dps[1], e.bias_dps[2], yaw_deg(&e) - y0);
}

/** @brief In flight mode the accelerometer is ignored: a 5 g sideways reading must not tilt the
 * attitude, while the pad mode would have pulled toward it. */
static void accel_lockout_case(void)
{
    const float zero[3] = {0}, side[3] = {5.0f, 0.0f, 1.0f};
    AtlasFlightEstimator e;
    make(&e, false);
    uint32_t t = pad(&e, 1000000U, 3.0f, zero);
    AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
    for (int i = 0; i < 480 * 5; ++i) { AtlasImuSample s = imu(t, side, zero); AtlasFlightEstimator_AddImu(&e, &s); t += 2083U; }
    CHECK(tilt_deg(&e) < 0.1f, "flight mode ignores the accelerometer");
    printf("  5 s of 5 g sideways in flight mode -> tilt %.3f deg\n", tilt_deg(&e));
}

/** @brief With the magnetometer, a gyro bias that appears in flight (not learned on the pad)
 * is held in check in heading, while gyro-only drifts. */
static void mag_case(void)
{
    const float zero[3] = {0}, up[3] = {0.0f, 0.0f, 1.0f}, bad[3] = {0.0f, 0.0f, 2.0f};
    float drift[2];
    for (int use = 0; use < 2; ++use)
    {
        AtlasFlightEstimator e;
        make(&e, use != 0);
        uint32_t t = 1000000U;
        AtlasMagSample m;
        memset(&m, 0, sizeof(m));
        m.field_gauss[0] = 0.0f; m.field_gauss[1] = 0.2f; m.field_gauss[2] = -0.4f; /* north, dipping down */
        m.flags = ATLAS_MAG_FLAG_VALID;
        for (int i = 0; i < 480 * 6; ++i)
        {
            if (i % 5 == 0) { m.timestamp_us = t; AtlasFlightEstimator_AddMag(&e, &m); }
            AtlasImuSample s = imu(t, up, zero); AtlasFlightEstimator_AddImu(&e, &s); t += 2083U;
        }
        AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
        const float y0 = yaw_deg(&e);
        for (int i = 0; i < 480 * 20; ++i)
        {
            if (i % 5 == 0) { m.timestamp_us = t; AtlasFlightEstimator_AddMag(&e, &m); }
            AtlasImuSample s = imu(t, up, bad); AtlasFlightEstimator_AddImu(&e, &s); t += 2083U;
        }
        drift[use] = yaw_deg(&e) - y0;
    }
    printf("  2 dps unlearned bias for 20 s: yaw drift %.2f deg gyro-only, %.2f deg with magnetometer\n",
           drift[0], drift[1]);
    CHECK(fabsf(drift[0]) > 35.0f, "gyro-only drifts");
    /* Fusion's heading loop leaves a steady offset of about bias / gain (2 dps / 0.5 rad/s ~ 4 deg). */
    CHECK(fabsf(drift[1]) < 6.0f, "magnetometer holds heading in flight mode");
}

/** @brief Velocity: 2 s of 6 g specific force straight up (5 g net) then 1 s of coast. */
/** @brief One 10 ms application cycle: about five IMU samples, then one ADXL375 sample.
 * @param e Estimator. @param t Microsecond clock (advanced). @param lsm LSM6 accel (g).
 * @param adxl ADXL375 reading (g, includes its offset). */
static void app_cycle(AtlasFlightEstimator *e, uint32_t *t, const float lsm[3], const float adxl[3])
{
    const float zero[3] = {0};
    const uint32_t end = *t + 10000U;
    while (*t + 2083U <= end) { *t += 2083U; AtlasImuSample s = imu(*t, lsm, zero); AtlasFlightEstimator_AddImu(e, &s); }
    *t = end;
    AtlasFlightEstimator_AddHighG(e, adxl, *t / 1000U);
}

/** @brief Speed from the ADXL375 along the nose: its axes are deliberately different from the
 * IMU's (the nose is its +X) and it has an offset on every axis. A 25 g boost (beyond the
 * LSM6's 16 g range) for 1 s, then 1 s of coast, at the real 100 Hz application cadence. */
static void velocity_case(void)
{
    const float off[3] = {0.2f, 0.1f, -0.1f}; /* ADXL375 zero-g offset, g */
    const float up[3] = {0.0f, 0.0f, 1.0f}, sat[3] = {0.0f, 0.0f, 16.0f}, none[3] = {0};
    const float adxl_pad[3] = {1.0f + off[0], off[1], off[2]};
    const float adxl_boost[3] = {25.0f + off[0], off[1], off[2]};
    const float adxl_coast[3] = {off[0], off[1], off[2]};
    AtlasFlightEstimator e;
    make(&e, false);
    uint32_t t = 1000000U;
    for (int i = 0; i < 600; ++i) app_cycle(&e, &t, up, adxl_pad); /* 6 s on the pad */
    CHECK(AtlasFlightEstimator_Speed(&e) < 0.01f, "zero speed on the pad");
    float nose[3];
    CHECK(AtlasFlightEstimator_NoseImu(&e, nose) && nose[2] > 0.999f, "nose axis from the pad");
    AtlasFlightEstimator_SetMode(&e, ATLAS_FEST_GYRO_MAG);
    for (int i = 0; i < 100; ++i) app_cycle(&e, &t, sat, adxl_boost);
    const float v_burnout = e.axial_velocity_m_s;
    for (int i = 0; i < 100; ++i) app_cycle(&e, &t, none, adxl_coast);
    printf("  speed along the nose (ADXL axes != IMU axes, offsets 0.2/0.1/-0.1 g): %.1f m/s at burnout "
           "(true 235.4), %.1f m/s 1 s later (true 225.6)\n", v_burnout, e.axial_velocity_m_s);
    /* Cross-axis offsets tilt the learned axis by ~7 deg: <1 % scale error, accepted as drift. */
    CHECK(fabsf(v_burnout - 235.4f) < 0.02f * 235.4f, "boost speed beyond LSM6 range");
    CHECK(fabsf(e.axial_velocity_m_s - 225.6f) < 0.02f * 225.6f, "coast speed");
    AtlasCanardInput in;
    CHECK(AtlasFlightEstimator_CanardInput(&e, AtlasFlightEstimator_Speed(&e), true, false, &in), "canard input");
    CHECK(fabsf(in.quaternion[0] * in.quaternion[0] + in.quaternion[1] * in.quaternion[1] +
                in.quaternion[2] * in.quaternion[2] + in.quaternion[3] * in.quaternion[3] - 1.0f) < 1e-5f, "unit q");
}

/** @brief Gaps and duplicates: a duplicate timestamp is ignored; a long gap is clamped and counted. */
static void gap_case(void)
{
    const float zero[3] = {0}, up[3] = {0.0f, 0.0f, 1.0f};
    AtlasFlightEstimator e;
    make(&e, false);
    uint32_t t = pad(&e, 1000000U, 3.0f, zero);
    AtlasImuSample s = imu(t - 2083U, up, zero);
    CHECK(!AtlasFlightEstimator_AddImu(&e, &s), "duplicate timestamp ignored");
    s = imu(t + 500000U, up, zero);
    CHECK(AtlasFlightEstimator_AddImu(&e, &s) && e.gaps == 1U, "0.5 s gap counted");
}

/** @brief Run all cases. @return Process status. */
int main(void)
{
    timing_case();
    bias_case();
    accel_lockout_case();
    mag_case();
    velocity_case();
    gap_case();
    if (failures) { printf("%d flight estimator check(s) failed\n", failures); return 1; }
    printf("Flight estimator tests passed\n");
    return 0;
}
