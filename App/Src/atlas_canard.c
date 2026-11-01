/**
 * @file atlas_canard.c
 * @brief Canard PID attitude control and moment-to-deflection allocation.
 *
 * Major functions:
 * - AtlasCanard_Init(): checks parameters and precomputes the allocation matrix.
 * - AtlasCanard_Step(): errors from the Fusion quaternion, PID, allocation, limits.
 * - canard_errors(): nose-pointing (pitch/yaw) error and roll twist about the nose.
 * - canard_roll_hold(): damp-then-hold roll mode with hysteresis.
 *
 * Allocation model: a canard at azimuth phi, radius R and arm l ahead of the CG
 * deflected by delta produces lift q*S*CLa*delta toward increasing azimuth, i.e. a
 * body moment q*S*CLa*delta*(-l cos(phi), -l sin(phi), R). The least-squares
 * (pseudo-inverse) solution spreads the requested moment over all canards.
 */
#include "atlas_canard.h"
#include <math.h>
#include <string.h>

#define CANARD_PI 3.14159265358979f
#define CANARD_RAD_PER_DEG (CANARD_PI / 180.0f)
#define CANARD_G_M_S2 9.80665f
#define CANARD_MAX_DT_S 0.05f

/** @brief Test a value for NaN/infinity. @param v Value. @return True if finite. */
static bool canard_finite(float v) { return isfinite(v) != 0; }
/** @brief Clamp a value. @param v Value. @param lo Lower bound. @param hi Upper bound.
 * @return Clamped value. */
static float canard_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
/** @brief Dot product. @param a Vector. @param b Vector. @return a.b */
static float canard_dot(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
/** @brief Wrap an angle into [-pi, pi]. @param a Angle (rad). @return Wrapped angle. */
static float canard_wrap(float a)
{
    while (a > CANARD_PI) a -= 2.0f * CANARD_PI;
    while (a < -CANARD_PI) a += 2.0f * CANARD_PI;
    return a;
}

/** @brief Cross product. @param a Vector. @param b Vector. @param out a x b (may not alias). */
static void canard_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/** @brief Hamilton product. @param a Left (w,x,y,z). @param b Right. @param out a*b (may not alias). */
static void canard_qmul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/** @brief Rotate v by unit q (q v q*), or by its inverse.
 * @param q Unit quaternion. @param inverse Use the conjugate. @param v Input. @param out Result. */
static void canard_rotate(const float q[4], bool inverse, const float v[3], float out[3])
{
    const float u[3] = {inverse ? -q[1] : q[1], inverse ? -q[2] : q[2], inverse ? -q[3] : q[3]};
    float t[3], c[3];
    canard_cross(u, v, t);
    for (unsigned i = 0U; i < 3U; ++i) t[i] *= 2.0f;
    canard_cross(u, t, c);
    for (unsigned i = 0U; i < 3U; ++i) out[i] = v[i] + q[0]*t[i] + c[i];
}

/** @brief Normalize a quaternion that should be unit length.
 * @param in Quaternion. @param out Normalized copy. @return False if non-finite or far from unit. */
static bool canard_unit(const float in[4], float out[4])
{
    const float n2 = in[0]*in[0] + in[1]*in[1] + in[2]*in[2] + in[3]*in[3];
    if (!canard_finite(n2) || n2 < 0.81f || n2 > 1.21f) return false;
    const float inv = 1.0f / sqrtf(n2);
    for (unsigned i = 0U; i < 4U; ++i) out[i] = in[i] * inv;
    return true;
}

/** @brief Body -> ENU attitude from a sample. @param s State. @param in Sample.
 * @param q_eb Result. @return False if the quaternion is unusable. */
static bool canard_body_attitude(const AtlasCanard *s, const AtlasCanardInput *in, float q_eb[4])
{
    float q_es[4], q_sb[4];
    if (!canard_unit(in->quaternion, q_es) || !canard_unit(s->config.sensor_from_body, q_sb)) return false;
    canard_qmul(q_es, q_sb, q_eb);
    return true;
}

/**
 * @brief Body-axis attitude errors relative to the target.
 * @param q_eb Current body -> ENU. @param target Desired body -> ENU.
 * @param max_tilt_rad Pointing-error clamp. @param error Out: X/Y pointing, Z roll (rad).
 * @return Angle between current and target nose directions (rad).
 */
static float canard_errors(const float q_eb[4], const float target[4], float max_tilt_rad, float error[3])
{
    static const float nose_body[3] = {0.0f, 0.0f, 1.0f};
    float nose[3], goal[3], axis[3], e_earth[3] = {0.0f, 0.0f, 0.0f}, e_body[3];
    canard_rotate(q_eb, false, nose_body, nose);
    canard_rotate(target, false, nose_body, goal);
    canard_cross(nose, goal, axis);
    const float sin_t = sqrtf(canard_dot(axis, axis));
    const float tilt = atan2f(sin_t, canard_dot(nose, goal));
    if (sin_t > 1e-6f)
    {
        const float scale = fminf(tilt, max_tilt_rad) / sin_t;
        for (unsigned i = 0U; i < 3U; ++i) e_earth[i] = axis[i] * scale;
    }
    else if (tilt > 1.0f) /* Exactly reversed: any perpendicular axis works; use body X. */
    {
        static const float x_body[3] = {1.0f, 0.0f, 0.0f};
        canard_rotate(q_eb, false, x_body, e_earth);
        for (unsigned i = 0U; i < 3U; ++i) e_earth[i] *= max_tilt_rad;
    }
    canard_rotate(q_eb, true, e_earth, e_body);

    /* Roll: twist of the body about its nose relative to the target attitude. */
    float conj_t[4] = {target[0], -target[1], -target[2], -target[3]}, q_err[4];
    canard_qmul(conj_t, q_eb, q_err);
    const float twist = canard_wrap(2.0f * atan2f(q_err[3], q_err[0]));

    error[0] = e_body[0];
    error[1] = e_body[1];
    error[2] = -twist;
    return tilt;
}

/** @brief Fill placeholder geometry (4 canards in a + layout) and starting gains.
 * @param c Destination. */
void AtlasCanard_DefaultConfig(AtlasCanardConfig *c)
{
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    c->sensor_from_body[0] = 1.0f;
    c->canard_count = 4U;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        c->canard[i].azimuth_deg = 90.0f * (float)i;
        c->canard[i].servo_center_us = 1500.0f;
        c->canard[i].servo_us_per_deg = 10.0f;
    }
    /* PLACEHOLDERS: replace with the real airframe once known. */
    c->canard_area_m2 = 0.002f;
    c->canard_lift_slope_per_rad = 4.0f;
    c->canard_arm_m = 0.8f;
    c->canard_radius_m = 0.06f;
    c->inertia_kg_m2[0] = 1.6f;
    c->inertia_kg_m2[1] = 1.6f;
    c->inertia_kg_m2[2] = 0.01f;
    c->air_density_kg_m3 = 1.225f;
    c->min_dynamic_pressure_pa = 500.0f; /* About 29 m/s at sea level. */
    /* Tilt: 6 rad/s natural frequency, damping 0.7. Roll: rate damping only. */
    c->kp_tilt = 36.0f;
    c->kd_tilt = 8.4f;
    c->ki_tilt = 0.0f;
    c->kp_roll = 0.0f;
    c->kd_roll = 10.0f;
    c->ki_roll = 0.0f;
    c->integral_limit_rad_s = 0.2f;
    c->max_tilt_error_deg = 20.0f;
    c->max_deflection_deg = 10.0f;
    c->max_rate_dps = 200.0f;
    c->min_pulse_us = 1000.0f;
    c->max_pulse_us = 2000.0f;
    c->launch_accel_g = 3.0f;
    c->launch_hold_us = 100000U;
    c->roll_hold_on_stop = false;
    c->roll_hold_enter_dps = 3.0f;
    c->roll_hold_exit_dps = 15.0f;
    c->roll_hold_settle_us = 200000U;
    c->soft_start_us = 0U;
    c->recovery_signal_off_us = 20000000U;
}

/** @brief Roll-only test setup: no pitch/yaw command, roll rate damped to zero, then the
 * angle where it stopped is held. Starts from AtlasCanard_DefaultConfig().
 * @param c Destination. */
void AtlasCanard_RollOnlyConfig(AtlasCanardConfig *c)
{
    if (c == NULL) return;
    AtlasCanard_DefaultConfig(c);
    c->kp_tilt = c->kd_tilt = c->ki_tilt = 0.0f; /* Only roll moment is ever requested. */
    c->roll_hold_on_stop = true;
    c->kd_roll = 10.0f;   /* Roll rate time constant 0.1 s. */
    c->kp_roll = 16.0f;   /* Hold stiffness once stopped: 4 rad/s, damping ratio 1.25. */
    c->ki_roll = 0.0f;
    c->soft_start_us = 1000000U; /* 1 s ramp from zero authority at every engage. */
}

/** @brief Build sensor_from_body from two directions measured in IMU axes.
 * @param nose_sensor Body +Z (nose) in sensor axes. @param x_hint_sensor Direction of canard
 * azimuth 0; only its part perpendicular to the nose is used.
 * @param sensor_from_body Result quaternion (w,x,y,z).
 * @return False if either direction is unusable or they are nearly parallel. */
bool AtlasCanard_MountFromAxes(const float nose_sensor[3], const float x_hint_sensor[3],
                               float sensor_from_body[4])
{
    if (nose_sensor == NULL || x_hint_sensor == NULL || sensor_from_body == NULL) return false;
    float z[3], x[3], y[3];
    const float zn = sqrtf(canard_dot(nose_sensor, nose_sensor));
    if (!canard_finite(zn) || zn < 1e-3f) return false;
    for (unsigned i = 0U; i < 3U; ++i) z[i] = nose_sensor[i] / zn;
    const float along = canard_dot(x_hint_sensor, z);
    for (unsigned i = 0U; i < 3U; ++i) x[i] = x_hint_sensor[i] - along * z[i];
    const float xn = sqrtf(canard_dot(x, x));
    if (!canard_finite(xn) || xn < 0.1f * sqrtf(canard_dot(x_hint_sensor, x_hint_sensor))) return false;
    for (unsigned i = 0U; i < 3U; ++i) x[i] /= xn;
    canard_cross(z, x, y);
    /* Columns of R are body X, Y, Z in sensor axes: v_sensor = R v_body. */
    const float r[3][3] = {{x[0], y[0], z[0]}, {x[1], y[1], z[1]}, {x[2], y[2], z[2]}};
    float q[4];
    const float tr = r[0][0] + r[1][1] + r[2][2];
    if (tr > 0.0f)
    {
        const float k = 2.0f * sqrtf(tr + 1.0f);
        q[0] = 0.25f * k; q[1] = (r[2][1] - r[1][2]) / k;
        q[2] = (r[0][2] - r[2][0]) / k; q[3] = (r[1][0] - r[0][1]) / k;
    }
    else if (r[0][0] > r[1][1] && r[0][0] > r[2][2])
    {
        const float k = 2.0f * sqrtf(1.0f + r[0][0] - r[1][1] - r[2][2]);
        q[0] = (r[2][1] - r[1][2]) / k; q[1] = 0.25f * k;
        q[2] = (r[0][1] + r[1][0]) / k; q[3] = (r[0][2] + r[2][0]) / k;
    }
    else if (r[1][1] > r[2][2])
    {
        const float k = 2.0f * sqrtf(1.0f + r[1][1] - r[0][0] - r[2][2]);
        q[0] = (r[0][2] - r[2][0]) / k; q[1] = (r[0][1] + r[1][0]) / k;
        q[2] = 0.25f * k; q[3] = (r[1][2] + r[2][1]) / k;
    }
    else
    {
        const float k = 2.0f * sqrtf(1.0f + r[2][2] - r[0][0] - r[1][1]);
        q[0] = (r[1][0] - r[0][1]) / k; q[1] = (r[0][2] + r[2][0]) / k;
        q[2] = (r[1][2] + r[2][1]) / k; q[3] = 0.25f * k;
    }
    return canard_unit(q, sensor_from_body);
}

/** @brief Check every scalar parameter. @param c Configuration. @return True if usable. */
static bool canard_config_valid(const AtlasCanardConfig *c)
{
    const float positive[] = {
        c->canard_area_m2, c->canard_lift_slope_per_rad, c->canard_radius_m,
        c->inertia_kg_m2[0], c->inertia_kg_m2[1], c->inertia_kg_m2[2], c->air_density_kg_m3,
        c->min_dynamic_pressure_pa, c->max_tilt_error_deg, c->max_deflection_deg,
        c->max_rate_dps, c->launch_accel_g};
    const float non_negative[] = {
        c->kp_tilt, c->kd_tilt, c->ki_tilt, c->kp_roll, c->kd_roll, c->ki_roll,
        c->integral_limit_rad_s};
    for (unsigned i = 0U; i < sizeof(positive) / sizeof(positive[0]); ++i)
        if (!canard_finite(positive[i]) || positive[i] <= 0.0f) return false;
    for (unsigned i = 0U; i < sizeof(non_negative) / sizeof(non_negative[0]); ++i)
        if (!canard_finite(non_negative[i]) || non_negative[i] < 0.0f) return false;
    if (c->canard_count < 3U || c->canard_count > ATLAS_CANARD_MAX) return false;
    if (!canard_finite(c->canard_arm_m) || fabsf(c->canard_arm_m) < 0.01f) return false;
    if (c->max_deflection_deg > 45.0f || c->max_tilt_error_deg > 90.0f ||
        c->max_rate_dps > ATLAS_CANARD_MAX_SLEW_DPS ||
        c->soft_start_us > ATLAS_CANARD_MAX_SOFT_START_US ||
        c->recovery_signal_off_us > ATLAS_CANARD_MAX_RECOVERY_SIGNAL_US) return false;
    if (c->roll_hold_on_stop &&
        (!canard_finite(c->roll_hold_enter_dps) || !canard_finite(c->roll_hold_exit_dps) ||
         c->roll_hold_enter_dps <= 0.0f || c->roll_hold_exit_dps <= c->roll_hold_enter_dps ||
         c->roll_hold_settle_us == 0U))
        return false;
    if (!canard_finite(c->min_pulse_us) || !canard_finite(c->max_pulse_us) ||
        c->min_pulse_us < 500.0f || c->max_pulse_us > 2500.0f || c->min_pulse_us >= c->max_pulse_us)
        return false;
    for (unsigned i = 0U; i < c->canard_count; ++i)
    {
        const AtlasCanardFin *f = &c->canard[i];
        if (!canard_finite(f->azimuth_deg) || !canard_finite(f->servo_center_us) ||
            !canard_finite(f->servo_us_per_deg) || f->servo_us_per_deg == 0.0f ||
            f->servo_center_us < c->min_pulse_us || f->servo_center_us > c->max_pulse_us)
            return false;
    }
    float q[4];
    return canard_unit(c->sensor_from_body, q);
}

/** @brief Validate a configuration and prepare the controller; target = nose up.
 * @param s State. @param c Configuration (copied).
 * @return False if any value is invalid or the canards cannot produce all three moments. */
bool AtlasCanard_Init(AtlasCanard *s, const AtlasCanardConfig *c)
{
    if (s == NULL || c == NULL) return false;
    memset(s, 0, sizeof(*s));
    if (!canard_config_valid(c)) return false;
    s->config = *c;
    canard_unit(c->sensor_from_body, s->config.sensor_from_body);

    /* G (3 x N): moment per unit deflection per q*S*CLa. allocation = G^T (G G^T)^-1. */
    float g[ATLAS_CANARD_MAX][3], a[3][3] = {{0}};
    for (unsigned j = 0U; j < c->canard_count; ++j)
    {
        const float phi = c->canard[j].azimuth_deg * CANARD_RAD_PER_DEG;
        g[j][0] = -c->canard_arm_m * cosf(phi);
        g[j][1] = -c->canard_arm_m * sinf(phi);
        g[j][2] = c->canard_radius_m;
        for (unsigned r = 0U; r < 3U; ++r)
            for (unsigned k = 0U; k < 3U; ++k) a[r][k] += g[j][r] * g[j][k];
    }
    const float det = a[0][0]*(a[1][1]*a[2][2] - a[1][2]*a[2][1]) -
                      a[0][1]*(a[1][0]*a[2][2] - a[1][2]*a[2][0]) +
                      a[0][2]*(a[1][0]*a[2][1] - a[1][1]*a[2][0]);
    /* Reject layouts that cannot independently control pitch, yaw and roll. */
    if (!(det > 1e-6f * a[0][0] * a[1][1] * a[2][2])) return false;
    const float inv[3][3] = {
        {(a[1][1]*a[2][2] - a[1][2]*a[2][1]) / det, (a[0][2]*a[2][1] - a[0][1]*a[2][2]) / det,
         (a[0][1]*a[1][2] - a[0][2]*a[1][1]) / det},
        {(a[1][2]*a[2][0] - a[1][0]*a[2][2]) / det, (a[0][0]*a[2][2] - a[0][2]*a[2][0]) / det,
         (a[0][2]*a[1][0] - a[0][0]*a[1][2]) / det},
        {(a[1][0]*a[2][1] - a[1][1]*a[2][0]) / det, (a[0][1]*a[2][0] - a[0][0]*a[2][1]) / det,
         (a[0][0]*a[1][1] - a[0][1]*a[1][0]) / det}};
    for (unsigned j = 0U; j < c->canard_count; ++j)
        for (unsigned k = 0U; k < 3U; ++k)
            s->allocation[j][k] = g[j][0]*inv[0][k] + g[j][1]*inv[1][k] + g[j][2]*inv[2][k];

    s->target[0] = 1.0f;
    s->configured = true;
    return true;
}

/** @brief Forget launch, speed, integrators and deflection history. @param s State. */
void AtlasCanard_Reset(AtlasCanard *s)
{
    if (s == NULL) return;
    memset(s->integral, 0, sizeof(s->integral));
    memset(s->velocity_m_s, 0, sizeof(s->velocity_m_s));
    memset(s->deflection_rad, 0, sizeof(s->deflection_rad));
    s->have_time = s->launched = s->launch_pending = s->saturated = false;
    s->roll_settling = s->roll_holding = s->was_active = false;
    s->deployed = s->signal_off = false;
}

/** @brief Hold a given body attitude. @param s State.
 * @param body_to_enu Desired attitude, w,x,y,z (normalized here). @return False if invalid. */
bool AtlasCanard_SetTarget(AtlasCanard *s, const float body_to_enu[4])
{
    float q[4];
    if (s == NULL || body_to_enu == NULL || !canard_unit(body_to_enu, q)) return false;
    memcpy(s->target, q, sizeof(q));
    memset(s->integral, 0, sizeof(s->integral));
    s->roll_settling = s->roll_holding = false;
    return true;
}

/** @brief Hold the attitude in this sample, e.g. on the launch rail.
 * @param s State. @param in Current sample. @return False if the sample is invalid. */
bool AtlasCanard_CaptureTarget(AtlasCanard *s, const AtlasCanardInput *in)
{
    float q_eb[4];
    if (s == NULL || in == NULL || !s->configured || !canard_body_attitude(s, in, q_eb)) return false;
    return AtlasCanard_SetTarget(s, q_eb);
}

/** @brief Latch launch and integrate earth acceleration into velocity.
 * @param s State. @param in Sample. @param dt Step (s). */
static void canard_track_flight(AtlasCanard *s, const AtlasCanardInput *in, float dt)
{
    const float boost = sqrtf(canard_dot(in->linear_accel_g, in->linear_accel_g));
    if (!s->launched)
    {
        if (boost < s->config.launch_accel_g) s->launch_pending = false;
        else if (!s->launch_pending) { s->launch_pending = true; s->launch_since_us = in->timestamp_us; }
        else if ((uint32_t)(in->timestamp_us - s->launch_since_us) >= s->config.launch_hold_us)
            s->launched = true;
    }
    if (s->launched)
        for (unsigned i = 0U; i < 3U; ++i) s->velocity_m_s[i] += in->earth_accel_g[i] * CANARD_G_M_S2 * dt;
}

/**
 * @brief Damp-then-hold roll: no angle command while spinning; once |rate| stays below the
 * enter threshold for the settle time, latch that angle and hold it until |rate| exceeds exit.
 * @param s State. @param twist Current roll relative to the target (rad).
 * @param rate_rad_s Body roll rate. @param now_us Sample time.
 * @return Roll error to use (rad): zero while damping, back to the latched angle while holding.
 */
static float canard_roll_hold(AtlasCanard *s, float twist, float rate_rad_s, uint32_t now_us)
{
    const AtlasCanardConfig *c = &s->config;
    const float rate_dps = fabsf(rate_rad_s) / CANARD_RAD_PER_DEG;
    if (s->roll_holding)
    {
        if (rate_dps > c->roll_hold_exit_dps) s->roll_holding = s->roll_settling = false;
    }
    else if (rate_dps < c->roll_hold_enter_dps)
    {
        if (!s->roll_settling) { s->roll_settling = true; s->roll_still_since_us = now_us; }
        else if ((uint32_t)(now_us - s->roll_still_since_us) >= c->roll_hold_settle_us)
        {
            s->roll_holding = true;
            s->roll_hold_twist = twist;
            s->integral[2] = 0.0f;
        }
    }
    else s->roll_settling = false;
    return s->roll_holding ? -canard_wrap(twist - s->roll_hold_twist) : 0.0f;
}

/** @brief Run one control update. Neutral (zero deflection, slew-limited) unless active.
 * @param s State. @param in Sample. @param out Result.
 * @return True when the canards are actively controlling. */
bool AtlasCanard_Step(AtlasCanard *s, const AtlasCanardInput *in, AtlasCanardOutput *out)
{
    if (s == NULL || in == NULL || out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!s->configured) { out->state = ATLAS_CANARD_NOT_CONFIGURED; return false; }
    const AtlasCanardConfig *c = &s->config;

    float dt = 0.0f;
    if (s->have_time) dt = fminf((float)(uint32_t)(in->timestamp_us - s->last_us) * 1e-6f, CANARD_MAX_DT_S);
    s->last_us = in->timestamp_us;
    s->have_time = true;

    /* Parachute deployment overrides everything and is never forgotten until Reset. */
    if (in->parachute_deployed && !s->deployed) { s->deployed = true; s->deployed_since_us = in->timestamp_us; }
    if (s->deployed && (uint32_t)(in->timestamp_us - s->deployed_since_us) >= c->recovery_signal_off_us)
        s->signal_off = true;

    bool finite = canard_finite(in->airspeed_m_s);
    for (unsigned i = 0U; i < 3U; ++i)
        finite = finite && canard_finite(in->earth_accel_g[i]) &&
                 canard_finite(in->linear_accel_g[i]) && canard_finite(in->gyro_dps[i]);
    float q_eb[4], desired[ATLAS_CANARD_MAX] = {0.0f};
    AtlasCanardState state = s->deployed ? ATLAS_CANARD_RECOVERY : ATLAS_CANARD_INVALID_INPUT;
    if (!s->deployed && finite && canard_body_attitude(s, in, q_eb))
    {
        canard_track_flight(s, in, dt);
        const float speed = in->airspeed_m_s >= 0.0f ? in->airspeed_m_s
                                                     : sqrtf(canard_dot(s->velocity_m_s, s->velocity_m_s));
        const float q_dyn = 0.5f * c->air_density_kg_m3 * speed * speed;
        out->speed_m_s = speed;
        out->dynamic_pressure_pa = q_dyn;
        out->launched = s->launched;

        float error[3], gyro_sensor[3], q_sb[4], rate[3];
        out->tilt_deg = canard_errors(q_eb, s->target, c->max_tilt_error_deg * CANARD_RAD_PER_DEG, error) /
                        CANARD_RAD_PER_DEG;
        for (unsigned i = 0U; i < 3U; ++i) gyro_sensor[i] = in->gyro_dps[i] * CANARD_RAD_PER_DEG;
        memcpy(q_sb, c->sensor_from_body, sizeof(q_sb));
        canard_rotate(q_sb, true, gyro_sensor, rate);
        state = !in->enable ? ATLAS_CANARD_DISABLED
              : q_dyn < c->min_dynamic_pressure_pa ? ATLAS_CANARD_LOW_PRESSURE : ATLAS_CANARD_ACTIVE;
        if (state == ATLAS_CANARD_ACTIVE && c->roll_hold_on_stop)
            error[2] = canard_roll_hold(s, -error[2], rate[2], in->timestamp_us);
        for (unsigned i = 0U; i < 3U; ++i)
        {
            out->error_deg[i] = error[i] / CANARD_RAD_PER_DEG;
            out->body_rate_dps[i] = rate[i] / CANARD_RAD_PER_DEG;
        }
        if (state == ATLAS_CANARD_ACTIVE)
        {
            if (!s->was_active) { s->was_active = true; s->active_since_us = in->timestamp_us; }
            float authority = 1.0f;
            if (c->soft_start_us != 0U)
                authority = fminf(1.0f, (float)(uint32_t)(in->timestamp_us - s->active_since_us) /
                                        (float)c->soft_start_us);
            out->authority = authority;
            const float kp[3] = {c->kp_tilt, c->kp_tilt, c->kp_roll};
            const float kd[3] = {c->kd_tilt, c->kd_tilt, c->kd_roll};
            const float ki[3] = {c->ki_tilt, c->ki_tilt, c->ki_roll};
            for (unsigned i = 0U; i < 3U; ++i)
            {
                if (!s->saturated && authority >= 1.0f && ki[i] > 0.0f)
                    s->integral[i] = canard_clamp(s->integral[i] + error[i] * dt,
                                                  -c->integral_limit_rad_s, c->integral_limit_rad_s);
                const float alpha = kp[i] * error[i] + ki[i] * s->integral[i] - kd[i] * rate[i];
                out->moment_nm[i] = c->inertia_kg_m2[i] * alpha;
            }
            /* Deflection = allocation * moment / (q S CLa); scale uniformly into the limit
               so the moment direction (pitch/yaw/roll mix) is preserved. */
            const float effectiveness = q_dyn * c->canard_area_m2 * c->canard_lift_slope_per_rad;
            const float limit = c->max_deflection_deg * CANARD_RAD_PER_DEG;
            float largest = 0.0f;
            for (unsigned j = 0U; j < c->canard_count; ++j)
            {
                desired[j] = canard_dot(s->allocation[j], out->moment_nm) / effectiveness;
                largest = fmaxf(largest, fabsf(desired[j]));
            }
            s->saturated = largest > limit;
            const float scale = (s->saturated ? limit / largest : 1.0f) * authority;
            for (unsigned j = 0U; j < c->canard_count; ++j) desired[j] *= scale;
        }
    }
    if (state != ATLAS_CANARD_ACTIVE)
    {
        memset(s->integral, 0, sizeof(s->integral));
        s->saturated = false;
        s->roll_settling = s->roll_holding = s->was_active = false;
    }

    const float max_step = c->max_rate_dps * CANARD_RAD_PER_DEG * dt;
    for (unsigned j = 0U; j < c->canard_count; ++j)
    {
        const float step = canard_clamp(desired[j] - s->deflection_rad[j], -max_step, max_step);
        s->deflection_rad[j] += step;
        const float deg = s->deflection_rad[j] / CANARD_RAD_PER_DEG;
        const float pulse = c->canard[j].servo_center_us + c->canard[j].servo_us_per_deg * deg;
        out->deflection_deg[j] = deg;
        out->pulse_us[j] = (uint16_t)lrintf(canard_clamp(pulse, c->min_pulse_us, c->max_pulse_us));
    }
    out->state = state;
    out->signal_off = s->signal_off;
    out->saturated = s->saturated;
    out->roll_holding = s->roll_holding;
    return state == ATLAS_CANARD_ACTIVE;
}
