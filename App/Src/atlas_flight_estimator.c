/**
 * @file atlas_flight_estimator.c
 * @brief Fusion AHRS (ENU) on per-sample LSM6DSV16B timestamps, mode-switched by flight phase.
 *
 * Major functions:
 * - AtlasFlightEstimator_AddImu(): pad bias learning, Fusion step with the measured interval,
 *   accelerometer only in CALIBRATE, magnetometer whenever fresh.
 * - AtlasFlightEstimator_AddHighG(): ADXL375 single-integrated speed along the pad nose axis.
 * - AtlasFlightEstimator_CanardInput(): normalized quaternion and derived accelerations.
 */
#include "atlas_flight_estimator.h"
#include <math.h>
#include <string.h>

#define FEST_G_M_S2 9.80665f
#define FEST_DEG_PER_RAD 57.29577951308232f

/** @brief Fill defaults. @param c Result. */
void AtlasFlightEstimator_DefaultConfig(AtlasFlightEstimatorConfig *c)
{
    static const float identity[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    memset(c, 0, sizeof(*c));
    c->sample_rate_hz = 480.0f;
    c->gain = 0.5f;
    c->accel_rejection_deg = 10.0f;
    c->mag_rejection_deg = 20.0f;
    c->rejection_timeout_s = 5.0f;
    c->gyro_range_dps = 4000.0f;
    c->use_magnetometer = true;
    memcpy(c->mag_to_imu, identity, sizeof(identity));
    c->mag_min_gauss = 0.2f;
    c->mag_max_gauss = 0.8f;
    c->mag_max_age_us = 50000U;
    c->bias_tau_s = 10.0f;
    c->bias_ready_s = 2.0f;
    c->still_gyro_dps = 5.0f;
    c->still_accel_g = 0.05f;
    c->max_dt_s = 0.05f;
    c->pad_velocity_tau_s = 1.0f;
    c->pad_tau_s = 5.0f;
    c->adxl_max_dt_s = 0.1f;
}

/** @brief Reset. @param e State. @param c Configuration. @return Validity. */
bool AtlasFlightEstimator_Init(AtlasFlightEstimator *e, const AtlasFlightEstimatorConfig *c)
{
    memset(e, 0, sizeof(*e));
    if (c == NULL || !(c->sample_rate_hz > 0.0f) || !(c->gain >= 0.0f) || !(c->max_dt_s > 0.0f) ||
        !(c->bias_tau_s > 0.0f) || !(c->pad_velocity_tau_s > 0.0f) || !(c->mag_max_gauss > c->mag_min_gauss))
        return false;
    e->config = *c;
    e->mode = ATLAS_FEST_CALIBRATE;
    return true;
}

/** @brief Switch mode. @param e State. @param m Mode. */
void AtlasFlightEstimator_SetMode(AtlasFlightEstimator *e, AtlasFlightEstimatorMode m) { e->mode = m; }

/** @brief Condition and store a magnetometer sample. @param e State. @param m Sample. */
void AtlasFlightEstimator_AddMag(AtlasFlightEstimator *e, const AtlasMagSample *m)
{
    const AtlasFlightEstimatorConfig *c = &e->config;
    if (!c->use_magnetometer || m == NULL || (m->flags & ATLAS_MAG_FLAG_VALID) == 0U) return;
    float raw[3], v[3];
    for (unsigned i = 0U; i < 3U; ++i) raw[i] = m->field_gauss[i] - c->mag_offset_gauss[i];
    for (unsigned r = 0U; r < 3U; ++r)
        v[r] = c->mag_to_imu[3U * r] * raw[0] + c->mag_to_imu[3U * r + 1U] * raw[1] + c->mag_to_imu[3U * r + 2U] * raw[2];
    const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!isfinite(n) || n < c->mag_min_gauss || n > c->mag_max_gauss) return;
    memcpy(e->mag_imu, v, sizeof(v));
    e->mag_us = m->timestamp_us;
    e->mag_valid = true;
}

/** @brief Start Fusion at the measured up direction. @param e State. @param a Accelerometer (g), |a| near 1. */
static void fest_start(AtlasFlightEstimator *e, const float a[3])
{
    const AtlasFlightEstimatorConfig *c = &e->config;
    FusionAhrsSettings settings = fusionAhrsDefaultSettings;
    settings.convention = FusionConventionEnu;
    settings.gain = c->gain;
    settings.gyroscopeRange = c->gyro_range_dps;
    settings.accelerationRejection = c->accel_rejection_deg;
    settings.magneticRejection = c->mag_rejection_deg;
    settings.rejectionTimeout = c->rejection_timeout_s;
    settings.sampleRate = c->sample_rate_hz;
    FusionAhrsInitialise(&e->ahrs);
    FusionAhrsSetSettings(&e->ahrs, &settings);
    /* Third row of the attitude matrix is "up" in sensor axes: q=(w,x,y,0) with
     * w=sqrt((1+uz)/2), x=uy/2w, y=-ux/2w reproduces the measured up exactly. */
    const float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float ux = a[0] / n, uy = a[1] / n, uz = a[2] / n;
    FusionQuaternion q = {.element = {0.0f, 1.0f, 0.0f, 0.0f}};
    if (uz > -0.999f)
    {
        const float w = sqrtf(0.5f * (1.0f + uz));
        q.element.w = w; q.element.x = uy / (2.0f * w); q.element.y = -ux / (2.0f * w); q.element.z = 0.0f;
    }
    FusionAhrsSetQuaternion(&e->ahrs, FusionQuaternionNormalise(q));
    /* With a magnetometer, keep Fusion's startup ramp so heading converges to the field. */
    if (!(c->use_magnetometer && e->mag_valid)) FusionAhrsSkipStartup(&e->ahrs);
    e->started = true;
}

/** @brief Normalized attitude and its rotation matrix (sensor -> ENU). @param e State.
 * @param q Quaternion out. @param r Matrix out. @return False if not finite. */
static bool fest_attitude(const AtlasFlightEstimator *e, FusionQuaternion *q, FusionMatrix *r)
{
    const FusionQuaternion raw = FusionAhrsGetQuaternion(&e->ahrs);
    const float n = FusionQuaternionNorm(raw);
    if (!isfinite(n) || n < 0.5f) return false;
    *q = FusionQuaternionScale(raw, 1.0f / n);
    *r = FusionQuaternionToMatrix(*q);
    return true;
}

/** @brief Fuse one IMU sample. @param e State. @param s Sample. @return True if fused. */
bool AtlasFlightEstimator_AddImu(AtlasFlightEstimator *e, const AtlasImuSample *s)
{
    const AtlasFlightEstimatorConfig *c = &e->config;
    const uint16_t need = ATLAS_IMU_FLAG_GYRO_VALID | ATLAS_IMU_FLAG_ACCEL_VALID;
    if (s == NULL || (s->flags & need) != need) return false;
    float a[3], w_raw[3];
    for (unsigned i = 0U; i < 3U; ++i)
    {
        a[i] = s->accel_m_s2[i] / FEST_G_M_S2;
        w_raw[i] = s->gyro_rad_s[i] * FEST_DEG_PER_RAD;
        if (!isfinite(a[i]) || !isfinite(w_raw[i])) return false;
    }
    if ((s->flags & ATLAS_IMU_FLAG_ACCEL_SATURATED) != 0U) ++e->accel_saturated;
    if ((s->flags & ATLAS_IMU_FLAG_GYRO_SATURATED) != 0U) ++e->gyro_saturated;

    /* Measured interval from the sensor timestamps; a gap is clamped, never extrapolated. */
    float dt = 1.0f / c->sample_rate_hz;
    if (e->have_time)
    {
        const uint32_t d_us = s->timestamp_us - e->last_us;
        if (d_us == 0U || d_us >= UINT32_C(0x80000000)) return false; /* Duplicate or out of order. */
        dt = (float)d_us * 1e-6f;
    }
    if ((s->flags & ATLAS_IMU_FLAG_GAP_BEFORE) != 0U || dt > c->max_dt_s) ++e->gaps;
    if (dt > c->max_dt_s) dt = c->max_dt_s;
    e->have_time = true;
    e->last_us = s->timestamp_us;

    const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float wn = sqrtf(w_raw[0] * w_raw[0] + w_raw[1] * w_raw[1] + w_raw[2] * w_raw[2]);
    /* The raw rate still contains the (up to ~10 dps) bias being learned, hence the allowance. */
    const bool still = fabsf(an - 1.0f) < c->still_accel_g && wn < c->still_gyro_dps + 10.0f;
    e->still = still;
    if (e->mode == ATLAS_FEST_CALIBRATE && still)
    {
        /* Pad gyro bias: average of the raw rate while still (bias is allowed up to ~10 dps). */
        const float k = dt / (c->bias_tau_s + dt);
        for (unsigned i = 0U; i < 3U; ++i)
            e->bias_dps[i] = (e->bias_still_s == 0.0f) ? w_raw[i] : e->bias_dps[i] + (w_raw[i] - e->bias_dps[i]) * k;
        e->bias_still_s += dt;
        /* Nose axis: the rocket stands nose-up on the pad, so mean "up" in IMU axes is the nose. */
        const float kp = dt / (c->pad_tau_s + dt);
        for (unsigned i = 0U; i < 3U; ++i)
            e->pad_up_imu[i] = (e->pad_still_s == 0.0f) ? a[i] / an : e->pad_up_imu[i] + (a[i] / an - e->pad_up_imu[i]) * kp;
        e->pad_still_s += dt;
    }
    for (unsigned i = 0U; i < 3U; ++i) { e->accel_g[i] = a[i]; e->gyro_dps[i] = w_raw[i] - e->bias_dps[i]; }

    if (!e->started)
    {
        if (e->mode != ATLAS_FEST_CALIBRATE || fabsf(an - 1.0f) > 0.15f) return false;
        fest_start(e, a);
        return true;
    }

    const bool mag_fresh = c->use_magnetometer && e->mag_valid &&
                           (uint32_t)(s->timestamp_us - e->mag_us + c->mag_max_age_us) <= 2U * c->mag_max_age_us;
    const FusionVector gyro = {.array = {e->gyro_dps[0], e->gyro_dps[1], e->gyro_dps[2]}};
    const FusionVector accel = (e->mode == ATLAS_FEST_CALIBRATE) ?
        (FusionVector){.array = {a[0], a[1], a[2]}} : FUSION_VECTOR_ZERO;
    const FusionVector mag = mag_fresh ?
        (FusionVector){.array = {e->mag_imu[0], e->mag_imu[1], e->mag_imu[2]}} : FUSION_VECTOR_ZERO;
    FusionAhrsSetSamplePeriod(&e->ahrs, dt);
    FusionAhrsUpdate(&e->ahrs, gyro, accel, mag);
    ++e->samples;
    if (mag_fresh) ++e->mag_used;
    return true;
}

/** @brief Nose direction in IMU axes. @param e State. @param nose Result. @return Validity. */
bool AtlasFlightEstimator_NoseImu(const AtlasFlightEstimator *e, float nose[3])
{
    const float *u = e->pad_up_imu;
    const float n = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (e->pad_still_s <= 0.0f || !isfinite(n) || n < 0.5f) return false;
    for (unsigned i = 0U; i < 3U; ++i) nose[i] = u[i] / n;
    return true;
}

/** @brief Single-integrate ADXL375 speed along the nose. @param e State.
 * @param g ADXL375 axes (g). @param t_ms Tick. @return True if integrated. */
bool AtlasFlightEstimator_AddHighG(AtlasFlightEstimator *e, const float g[3], uint32_t t_ms)
{
    const AtlasFlightEstimatorConfig *c = &e->config;
    FusionQuaternion q;
    FusionMatrix r;
    float nose[3];
    if (g == NULL || !isfinite(g[0]) || !isfinite(g[1]) || !isfinite(g[2]) || !e->started) return false;
    if (!e->have_adxl || t_ms == e->adxl_ms)
    {
        e->have_adxl = true;
        e->adxl_ms = t_ms;
        return false;
    }
    float dt = (float)(t_ms - e->adxl_ms) * 0.001f;
    e->adxl_ms = t_ms;
    if (dt > c->adxl_max_dt_s) dt = c->adxl_max_dt_s;

    const bool pad = e->mode == ATLAS_FEST_CALIBRATE;
    if (pad && e->still)
    {
        /* Standing still nose-up, the ADXL375 reads 1 g along the nose plus its offset. */
        const float k = dt / (c->pad_tau_s + dt);
        const bool first = e->adxl_pad_g[0] == 0.0f && e->adxl_pad_g[1] == 0.0f && e->adxl_pad_g[2] == 0.0f;
        for (unsigned i = 0U; i < 3U; ++i)
            e->adxl_pad_g[i] = first ? g[i] : e->adxl_pad_g[i] + (g[i] - e->adxl_pad_g[i]) * k;
    }
    const float *m = e->adxl_pad_g;
    const float mn = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (!(mn > 0.5f) || !AtlasFlightEstimator_NoseImu(e, nose) || !fest_attitude(e, &q, &r)) return false;

    /* Along the nose: ADXL375 projected on its pad gravity direction, minus its axial offset
     * (|pad mean| - 1 g), minus gravity's component cos(tilt) from the attitude. */
    e->adxl_axial_g = (g[0] * m[0] + g[1] * m[1] + g[2] * m[2]) / mn - (mn - 1.0f);
    e->cos_tilt = nose[0] * r.element.zx + nose[1] * r.element.zy + nose[2] * r.element.zz;
    e->axial_velocity_m_s += (e->adxl_axial_g - e->cos_tilt) * FEST_G_M_S2 * dt;
    if (pad)
    {
        /* On the pad it leaks to zero (and resets when still), so it only carries the first
         * instants of boost before ENGINE_ON is confirmed. */
        e->axial_velocity_m_s -= e->axial_velocity_m_s * dt / (c->pad_velocity_tau_s + dt);
        if (e->still) e->axial_velocity_m_s = 0.0f;
    }
    return true;
}

/** @brief Ready for flight. @param e State. @return True when calibrated and converged. */
bool AtlasFlightEstimator_Ready(const AtlasFlightEstimator *e)
{
    return e->started && !FusionAhrsGetFlags(&e->ahrs).startup && e->bias_still_s >= e->config.bias_ready_s;
}

/** @brief Speed. @param e State. @return m/s. */
float AtlasFlightEstimator_Speed(const AtlasFlightEstimator *e)
{
    return fabsf(e->axial_velocity_m_s);
}

/** @brief Canard input from the latest sample. @param e State. @param airspeed_m_s Airspeed.
 * @param enable Permission. @param parachute_deployed Recovery event. @param in Result.
 * @return False without an attitude. */
bool AtlasFlightEstimator_CanardInput(const AtlasFlightEstimator *e, float airspeed_m_s, bool enable,
                                      bool parachute_deployed, AtlasCanardInput *in)
{
    memset(in, 0, sizeof(*in));
    FusionQuaternion q;
    FusionMatrix r;
    if (!e->started || !fest_attitude(e, &q, &r)) return false;
    const FusionVector accel = {.array = {e->accel_g[0], e->accel_g[1], e->accel_g[2]}};
    const FusionVector up = {.array = {r.element.zx, r.element.zy, r.element.zz}};
    const FusionVector linear = FusionVectorSubtract(accel, up);
    FusionVector earth = FusionMatrixMultiply(r, accel);
    earth.axis.z -= 1.0f;
    memcpy(in->quaternion, q.array, sizeof(in->quaternion));
    memcpy(in->linear_accel_g, linear.array, sizeof(in->linear_accel_g));
    memcpy(in->earth_accel_g, earth.array, sizeof(in->earth_accel_g));
    memcpy(in->gyro_dps, e->gyro_dps, sizeof(in->gyro_dps));
    in->airspeed_m_s = airspeed_m_s;
    in->timestamp_us = e->last_us;
    in->enable = enable;
    in->parachute_deployed = parachute_deployed;
    return true;
}
