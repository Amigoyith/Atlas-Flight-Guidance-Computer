/**
 * @file test_canard.c
 * @brief Host tests for the canard controller: allocation, gates and closed-loop flight.
 *
 * Major functions:
 * - allocation_cases(): pseudo-inverse reproduces moments for +, X and 3-canard layouts.
 * - gate_cases(): disable, low dynamic pressure, invalid input, saturation, slew, pulses.
 * - flight_case(): rigid-body rocket with servo lag converges to the target attitude.
 *
 * The plant uses the same canard lift model as the controller, scaled by a mismatch
 * factor to show tolerance to wrong airframe numbers. It has no aerodynamic
 * restoring moment (a neutrally stable rocket), which is the harder case.
 */
#include "atlas_canard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define RAD(d) ((d) * PI_F / 180.0f)
#define DEG(r) ((r) * 180.0f / PI_F)

/** @brief Hamilton product. @param a Left. @param b Right. @param o Result. */
static void qmul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/** @brief Axis-angle quaternion. @param axis Unit axis. @param deg Angle. @param q Result. */
static void qaxis(const float axis[3], float deg, float q[4])
{
    const float h = RAD(deg) * 0.5f;
    q[0] = cosf(h); q[1] = axis[0]*sinf(h); q[2] = axis[1]*sinf(h); q[3] = axis[2]*sinf(h);
}

/** @brief Moment produced by deflections, using the controller's lift model.
 * @param c Config. @param q_dyn Dynamic pressure. @param scale Plant/model mismatch.
 * @param defl_deg Deflections. @param m Result moment. */
static void plant_moment(const AtlasCanardConfig *c, float q_dyn, float scale,
                         const float *defl_deg, float m[3])
{
    const float k = q_dyn * c->canard_area_m2 * c->canard_lift_slope_per_rad * scale;
    m[0] = m[1] = m[2] = 0.0f;
    for (unsigned j = 0U; j < c->canard_count; ++j)
    {
        const float phi = RAD(c->canard[j].azimuth_deg), d = RAD(defl_deg[j]);
        m[0] += k * d * -c->canard_arm_m * cosf(phi);
        m[1] += k * d * -c->canard_arm_m * sinf(phi);
        m[2] += k * d * c->canard_radius_m;
    }
}

/** @brief Sample with a given attitude and rates. @param q Sensor->ENU.
 * @param rate_dps Gyro. @param t_us Time. @param in Result. */
static void sample(const float q[4], const float rate_dps[3], uint32_t t_us, AtlasCanardInput *in)
{
    memset(in, 0, sizeof(*in));
    memcpy(in->quaternion, q, sizeof(in->quaternion));
    memcpy(in->gyro_dps, rate_dps, sizeof(in->gyro_dps));
    in->airspeed_m_s = 100.0f;
    in->timestamp_us = t_us;
    in->enable = true;
}

/** @brief Pseudo-inverse allocation reproduces requested moments exactly. */
static void allocation_cases(void)
{
    const float layouts[3][4] = {{0, 90, 180, 270}, {45, 135, 225, 315}, {90, 210, 330, 0}};
    const unsigned counts[3] = {4U, 4U, 3U};
    for (unsigned l = 0U; l < 3U; ++l)
    {
        AtlasCanardConfig c;
        AtlasCanard s;
        AtlasCanardOutput out;
        AtlasCanardInput in;
        AtlasCanard_DefaultConfig(&c);
        c.canard_count = counts[l];
        for (unsigned j = 0U; j < counts[l]; ++j) c.canard[j].azimuth_deg = layouts[l][j];
        c.max_deflection_deg = 45.0f;
        c.max_rate_dps = ATLAS_CANARD_MAX_SLEW_DPS;
        assert(AtlasCanard_Init(&s, &c));
        /* Small tilt about X plus a roll rate: all three moment axes requested. */
        float q[4];
        const float axis[3] = {0.6f, 0.8f, 0.0f}, rate[3] = {0.0f, 0.0f, 30.0f};
        qaxis(axis, 2.0f, q);
        sample(q, rate, 0U, &in);
        AtlasCanard_Step(&s, &in, &out);
        for (uint32_t t = 2000U; t <= 200000U; t += 2000U) /* let the slew limit settle */
        {
            in.timestamp_us = t;
            assert(AtlasCanard_Step(&s, &in, &out) && !out.saturated);
        }
        float m[3];
        plant_moment(&c, out.dynamic_pressure_pa, 1.0f, out.deflection_deg, m);
        for (unsigned i = 0U; i < 3U; ++i)
            assert(fabsf(m[i] - out.moment_nm[i]) < 1e-3f + 1e-3f * fabsf(out.moment_nm[i]));
        assert(fabsf(out.moment_nm[0]) > 0.0f && fabsf(out.moment_nm[1]) > 0.0f && out.moment_nm[2] < 0.0f);
    }
    /* Two canards cannot make three independent moments. */
    AtlasCanardConfig c;
    AtlasCanard s;
    AtlasCanard_DefaultConfig(&c);
    c.canard_count = 3U;
    c.canard[0].azimuth_deg = 0.0f; c.canard[1].azimuth_deg = 180.0f; c.canard[2].azimuth_deg = 0.0f;
    assert(!AtlasCanard_Init(&s, &c));
    AtlasCanard_DefaultConfig(&c);
    c.canard_area_m2 = 0.0f;
    assert(!AtlasCanard_Init(&s, &c));
    puts("Canard allocation: +, X and 3-canard layouts reproduce moments; degenerate rejected PASS");
}

/** @brief Gates, saturation, slew limit, servo mapping and launch/speed estimate. */
static void gate_cases(void)
{
    AtlasCanardConfig c;
    AtlasCanard s;
    AtlasCanardOutput out;
    AtlasCanardInput in;
    AtlasCanard_DefaultConfig(&c);
    c.canard[0].servo_us_per_deg = -10.0f;
    assert(AtlasCanard_Init(&s, &c));
    const float x_axis[3] = {1.0f, 0.0f, 0.0f}, still[3] = {0};
    float q[4];
    qaxis(x_axis, 60.0f, q);

    /* Disabled: neutral pulses, no motion. */
    sample(q, still, 0U, &in);
    in.enable = false;
    assert(!AtlasCanard_Step(&s, &in, &out) && out.state == ATLAS_CANARD_DISABLED);
    for (unsigned j = 0U; j < 4U; ++j) assert(out.pulse_us[j] == 1500U && out.deflection_deg[j] == 0.0f);

    /* Large error saturates at the limit; the slew cap applies first (max_rate_dps * 2 ms). */
    in.enable = true;
    in.timestamp_us = 2000U;
    assert(AtlasCanard_Step(&s, &in, &out) && out.saturated);
    float largest = 0.0f;
    for (unsigned j = 0U; j < 4U; ++j) largest = fmaxf(largest, fabsf(out.deflection_deg[j]));
    assert(fabsf(largest - c.max_rate_dps * 0.002f) < 1e-3f);
    for (uint32_t t = 4000U; t <= 200000U; t += 2000U) { in.timestamp_us = t; AtlasCanard_Step(&s, &in, &out); }
    largest = 0.0f;
    for (unsigned j = 0U; j < 4U; ++j) largest = fmaxf(largest, fabsf(out.deflection_deg[j]));
    assert(fabsf(largest - c.max_deflection_deg) < 1e-3f);
    /* Tilt about +X needs lift along Y: canards 0 and 2 (on +/-X) act; servo 0 is reversed. */
    assert(fabsf(out.deflection_deg[1]) < 1e-3f && fabsf(out.deflection_deg[3]) < 1e-3f);
    assert(out.pulse_us[0] == (uint16_t)lrintf(1500.0f - 10.0f * out.deflection_deg[0]));
    assert(fabsf(out.tilt_deg - 60.0f) < 0.01f && fabsf(out.error_deg[0] + 20.0f) < 0.01f);

    /* Low dynamic pressure returns to neutral through the slew limit. */
    in.airspeed_m_s = 10.0f;
    for (uint32_t t = 202000U; t <= 400000U; t += 2000U) { in.timestamp_us = t; AtlasCanard_Step(&s, &in, &out); }
    assert(out.state == ATLAS_CANARD_LOW_PRESSURE);
    for (unsigned j = 0U; j < 4U; ++j) assert(out.deflection_deg[j] == 0.0f && out.pulse_us[j] == 1500U);

    /* Non-finite input is rejected. */
    in.airspeed_m_s = 100.0f;
    in.gyro_dps[0] = NAN;
    in.timestamp_us = 402000U;
    assert(!AtlasCanard_Step(&s, &in, &out) && out.state == ATLAS_CANARD_INVALID_INPUT);
    in.gyro_dps[0] = 0.0f;
    in.quaternion[0] = 0.0f; in.quaternion[1] = 0.0f; in.quaternion[2] = 0.0f; in.quaternion[3] = 0.0f;
    assert(!AtlasCanard_Step(&s, &in, &out) && out.state == ATLAS_CANARD_INVALID_INPUT);

    /* Launch latch needs sustained boost; then speed integrates earth acceleration. */
    AtlasCanard_Reset(&s);
    const float level[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    sample(level, still, 0U, &in);
    in.airspeed_m_s = -1.0f;
    in.linear_accel_g[2] = 5.0f;
    in.earth_accel_g[2] = 5.0f;
    for (uint32_t t = 0U; t <= 50000U; t += 2000U) { in.timestamp_us = t; AtlasCanard_Step(&s, &in, &out); }
    assert(!out.launched);
    in.linear_accel_g[2] = 1.0f;
    in.timestamp_us = 52000U;
    AtlasCanard_Step(&s, &in, &out);
    in.linear_accel_g[2] = 5.0f;
    for (uint32_t t = 54000U; t <= 1154000U; t += 2000U) { in.timestamp_us = t; AtlasCanard_Step(&s, &in, &out); }
    assert(out.launched);
    /* Latched at 154 ms; 1.0 s of 5 g afterwards is about 49 m/s. */
    assert(fabsf(out.speed_m_s - 5.0f * 9.80665f * 1.0f) < 1.5f);
    assert(out.state == ATLAS_CANARD_ACTIVE);
    puts("Canard gates: disable, saturation, slew, reversed servo, low q, invalid input, launch/speed PASS");
}

/** @brief Rotate v by q (q v q*). @param q Unit quaternion. @param v Input. @param o Result. */
static void qrot(const float q[4], const float v[3], float o[3])
{
    const float p[4] = {0.0f, v[0], v[1], v[2]}, qc[4] = {q[0], -q[1], -q[2], -q[3]};
    float t[4], r[4];
    qmul(q, p, t);
    qmul(t, qc, r);
    o[0] = r[1]; o[1] = r[2]; o[2] = r[3];
}

/** @brief Parachute deployment: neutral at once, neutral signal for 20 s, then signal off. */
static void recovery_case(void)
{
    AtlasCanardConfig c;
    AtlasCanard s;
    AtlasCanardOutput out;
    AtlasCanardInput in;
    AtlasCanard_RollOnlyConfig(&c);
    c.soft_start_us = 0U;
    AtlasCanardConfig bad = c;
    bad.recovery_signal_off_us = ATLAS_CANARD_MAX_RECOVERY_SIGNAL_US + 1U;
    assert(!AtlasCanard_Init(&s, &bad));
    assert(AtlasCanard_Init(&s, &c) && c.recovery_signal_off_us == 20000000U);
    const float level[4] = {1.0f, 0.0f, 0.0f, 0.0f}, spin[3] = {0.0f, 0.0f, 400.0f};
    uint32_t t = 0U;
    for (; t <= 300000U; t += 2000U) { sample(level, spin, t, &in); AtlasCanard_Step(&s, &in, &out); }
    assert(out.state == ATLAS_CANARD_ACTIVE && fabsf(out.deflection_deg[0]) > 1.0f);
    const uint32_t deploy = t;
    for (; t <= deploy + 21000000U; t += 2000U)
    {
        sample(level, spin, t, &in);
        in.parachute_deployed = (t - deploy) < 1000000U;  /* event seen only briefly */
        AtlasCanard_Step(&s, &in, &out);
        assert(out.state == ATLAS_CANARD_RECOVERY);
        if (t - deploy >= 100000U) assert(out.deflection_deg[0] == 0.0f && out.pulse_us[0] == 1500U);
        assert(out.signal_off == ((t - deploy) >= 20000000U));
    }
    AtlasCanard_Reset(&s);
    sample(level, spin, t, &in);
    AtlasCanard_Step(&s, &in, &out);
    assert(!out.signal_off && out.state == ATLAS_CANARD_ACTIVE);
    puts("Canard recovery: parachute -> neutral at once, neutral signal 20 s, then latched signal off PASS");
}

/** @brief Soft start: linear authority ramp at every engage, then identical to no ramp. */
static void soft_start_case(void)
{
    AtlasCanardConfig c, h;
    AtlasCanard soft, hard;
    AtlasCanardOutput so, ho;
    AtlasCanardInput in;
    AtlasCanard_RollOnlyConfig(&c);
    c.roll_hold_on_stop = false;
    c.kp_roll = 0.0f;                        /* pure rate damping: constant command */
    c.max_rate_dps = ATLAS_CANARD_MAX_SLEW_DPS;
    h = c;
    h.soft_start_us = 0U;
    AtlasCanardConfig bad = c;
    bad.soft_start_us = ATLAS_CANARD_MAX_SOFT_START_US + 1U;
    assert(!AtlasCanard_Init(&soft, &bad));
    assert(AtlasCanard_Init(&soft, &c) && AtlasCanard_Init(&hard, &h));
    const float level[4] = {1.0f, 0.0f, 0.0f, 0.0f}, spin[3] = {0.0f, 0.0f, 30.0f};
    for (uint32_t t = 0U; t <= 1500000U; t += 2000U)
    {
        sample(level, spin, t, &in);
        AtlasCanard_Step(&soft, &in, &so);
        AtlasCanard_Step(&hard, &in, &ho);
        if (t == 500000U)
            assert(fabsf(so.authority - 0.5f) < 1e-3f &&
                   fabsf(so.deflection_deg[0] / ho.deflection_deg[0] - 0.5f) < 0.01f);
        if (t >= 1200000U)
            assert(so.authority == 1.0f && fabsf(so.deflection_deg[0] - ho.deflection_deg[0]) < 1e-5f);
    }
    in.enable = false;
    in.timestamp_us = 1502000U;
    AtlasCanard_Step(&soft, &in, &so);       /* disengage... */
    in.enable = true;
    in.timestamp_us = 1504000U;
    AtlasCanard_Step(&soft, &in, &so);       /* ...re-engage restarts the ramp */
    assert(so.authority < 1e-3f);
    puts("Canard soft start: linear authority ramp, identical afterwards, restarts on re-engage PASS");
}

/** @brief Roll-only mode: no pitch/yaw command, damp, latch, hold, re-damp, slew cap, mount. */
static void roll_only_cases(void)
{
    AtlasCanardConfig c;
    AtlasCanard s;
    AtlasCanardOutput out;
    AtlasCanardInput in;
    AtlasCanard_RollOnlyConfig(&c);
    AtlasCanardConfig bad = c;
    bad.max_rate_dps = ATLAS_CANARD_MAX_SLEW_DPS + 1.0f;
    assert(!AtlasCanard_Init(&s, &bad));                    /* slew cap is a hard ceiling */
    bad = c;
    bad.roll_hold_exit_dps = bad.roll_hold_enter_dps;
    assert(!AtlasCanard_Init(&s, &bad));                    /* hysteresis required */
    assert(AtlasCanard_Init(&s, &c));

    /* Tilted 5 degrees and spinning 120 dps. Plant: same lift model, 20 ms servo lag. */
    const float x_axis[3] = {1.0f, 0.0f, 0.0f};
    float q_eb[4], omega[3] = {0.0f, 0.0f, RAD(120.0f)}, servo[ATLAS_CANARD_MAX] = {0};
    qaxis(x_axis, 5.0f, q_eb);
    const float dt = 0.002f, tau = 0.02f, max_step = c.max_rate_dps * dt + 1e-4f;
    float previous[ATLAS_CANARD_MAX] = {0}, held_twist = 0.0f, worst_spread = 0.0f;
    bool latched = false, relatched = false;
    float twist_at_latch = 0.0f, twist_end = 0.0f, final_tilt = 0.0f;
    for (unsigned n = 0U; n < 4000U; ++n) /* 8 s */
    {
        float disturb = 0.0f;
        if (n >= 1500U && n < 1525U) disturb = 0.02f;  /* 50 ms nudge, below the exit rate */
        if (n >= 2500U && n < 2600U) disturb = 0.2f;   /* 200 ms shove: spins it up again */
        const float rate_dps[3] = {DEG(omega[0]), DEG(omega[1]), DEG(omega[2])};
        sample(q_eb, rate_dps, n * 2000U, &in);
        AtlasCanard_Step(&s, &in, &out);
        float mean = 0.0f;
        for (unsigned j = 0U; j < c.canard_count; ++j)
        {
            assert(fabsf(out.deflection_deg[j] - previous[j]) <= max_step); /* slew cap */
            previous[j] = out.deflection_deg[j];
            mean += out.deflection_deg[j] / (float)c.canard_count;
        }
        for (unsigned j = 0U; j < c.canard_count; ++j)
            worst_spread = fmaxf(worst_spread, fabsf(out.deflection_deg[j] - mean)); /* roll only */
        const float conj_t[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float q_err[4];
        qmul(conj_t, q_eb, q_err);
        const float twist = DEG(2.0f * atan2f(q_err[3], q_err[0]));
        if (out.roll_holding && !latched) { latched = true; twist_at_latch = twist; held_twist = twist; }
        if (n > 1525U && n < 2500U) assert(out.roll_holding); /* nudge does not break the hold */
        if (n == 2499U) assert(fabsf(twist - held_twist) < 1.0f); /* and it returned to the angle */
        if (n >= 2600U && out.roll_holding && fabsf(twist - held_twist) > 5.0f) relatched = true;
        twist_end = twist;
        final_tilt = out.tilt_deg;

        float m[3];
        for (unsigned j = 0U; j < c.canard_count; ++j) servo[j] += (out.deflection_deg[j] - servo[j]) * dt / tau;
        plant_moment(&c, out.dynamic_pressure_pa, 1.0f, servo, m);
        m[2] += disturb;
        for (unsigned i = 0U; i < 3U; ++i) omega[i] += m[i] / c.inertia_kg_m2[i] * dt;
        const float w[4] = {0.0f, omega[0], omega[1], omega[2]};
        float dq[4], norm = 0.0f;
        qmul(q_eb, w, dq);
        for (unsigned i = 0U; i < 4U; ++i) { q_eb[i] += 0.5f * dq[i] * dt; norm += q_eb[i] * q_eb[i]; }
        for (unsigned i = 0U; i < 4U; ++i) q_eb[i] /= sqrtf(norm);
    }
    printf("  roll-only: latched at %.1f deg, after shove re-latched at %.1f deg, final rate %.3f dps, "
           "tilt left at %.2f deg, worst non-roll spread %.5f deg\n", (double)twist_at_latch,
           (double)twist_end, (double)DEG(omega[2]), (double)final_tilt, (double)worst_spread);
    assert(latched && relatched && out.roll_holding && fabsf(DEG(omega[2])) < 0.5f);
    assert(worst_spread < 1e-3f);                 /* identical deflections: pure roll command */
    assert(fabsf(final_tilt - 5.0f) < 0.5f);      /* pitch/yaw left alone */

    /* Mounting helper: nose along sensor -Y, azimuth 0 along sensor +Z. */
    const float nose[3] = {0.0f, -1.0f, 0.0f}, hint[3] = {0.0f, 0.3f, 1.0f}; /* tilted toward the nose */
    const float bz[3] = {0.0f, 0.0f, 1.0f}, bx[3] = {1.0f, 0.0f, 0.0f};
    float mount[4], vz[3], vx[3];
    assert(AtlasCanard_MountFromAxes(nose, hint, mount));
    qrot(mount, bz, vz);
    qrot(mount, bx, vx);
    assert(fabsf(vz[1] + 1.0f) < 1e-5f && fabsf(vx[2] - 1.0f) < 1e-5f);
    assert(!AtlasCanard_MountFromAxes(nose, nose, mount));
    puts("Canard roll-only: damp, latch, hold through a nudge, re-latch after a shove, slew cap, no pitch/yaw, mount PASS");
}

/**
 * @brief Fly a rigid-body rocket from a tilted, rolling start.
 * @param mismatch Real canard effectiveness / modelled effectiveness.
 * @param mount Sensor mounting relative to body (w,x,y,z).
 * @param settle_s Time allowed to settle.
 * @param hold_roll Also hold roll angle (kp_roll > 0), starting 30 degrees off.
 */
static void flight_case(float mismatch, const float mount[4], float settle_s, bool hold_roll)
{
    AtlasCanardConfig c;
    AtlasCanard s;
    AtlasCanardOutput out;
    AtlasCanardInput in;
    AtlasCanard_DefaultConfig(&c);
    memcpy(c.sensor_from_body, mount, sizeof(c.sensor_from_body));
    if (hold_roll) { c.kp_roll = 16.0f; c.kd_roll = 5.6f; }
    assert(AtlasCanard_Init(&s, &c));

    const float axis[3] = {0.6f, 0.8f, 0.0f}, nose[3] = {0.0f, 0.0f, 1.0f};
    float q_eb[4], q_tilt[4], q_roll[4], omega[3] = {0.0f, 0.0f, RAD(90.0f)};
    float servo[ATLAS_CANARD_MAX] = {0};
    qaxis(axis, 10.0f, q_tilt);
    qaxis(nose, hold_roll ? 30.0f : 0.0f, q_roll);
    qmul(q_tilt, q_roll, q_eb);
    const float dt = 0.002f, tau = 0.02f;
    float worst_late_tilt = 0.0f, worst_late_roll = 0.0f, peak_defl = 0.0f;
    const unsigned steps = (unsigned)(settle_s / dt) + 500U;
    for (unsigned n = 0U; n < steps; ++n)
    {
        /* IMU sees sensor->ENU = body->ENU * inverse(sensor_from_body). */
        const float mount_conj[4] = {mount[0], -mount[1], -mount[2], -mount[3]};
        float q_es[4], gyro_sensor[3], w_dps[3];
        qmul(q_eb, mount_conj, q_es);
        for (unsigned i = 0U; i < 3U; ++i) w_dps[i] = DEG(omega[i]);
        { /* rotate body rate into sensor axes: v_s = mount * v_b * mount^-1 */
            const float v[4] = {0.0f, w_dps[0], w_dps[1], w_dps[2]};
            float t[4], r[4];
            qmul(mount, v, t);
            qmul(t, mount_conj, r);
            gyro_sensor[0] = r[1]; gyro_sensor[1] = r[2]; gyro_sensor[2] = r[3];
        }
        sample(q_es, gyro_sensor, n * 2000U, &in);
        AtlasCanard_Step(&s, &in, &out);

        float m[3];
        for (unsigned j = 0U; j < c.canard_count; ++j)
        {
            servo[j] += (out.deflection_deg[j] - servo[j]) * dt / tau;
            peak_defl = fmaxf(peak_defl, fabsf(servo[j]));
        }
        plant_moment(&c, out.dynamic_pressure_pa, mismatch, servo, m);
        const float *I = c.inertia_kg_m2;
        const float gyro_term[3] = {
            (I[2] - I[1]) * omega[1] * omega[2], (I[0] - I[2]) * omega[2] * omega[0],
            (I[1] - I[0]) * omega[0] * omega[1]};
        for (unsigned i = 0U; i < 3U; ++i) omega[i] += (m[i] - gyro_term[i]) / I[i] * dt;
        const float w[4] = {0.0f, omega[0], omega[1], omega[2]};
        float dq[4];
        qmul(q_eb, w, dq);
        float norm = 0.0f;
        for (unsigned i = 0U; i < 4U; ++i) { q_eb[i] += 0.5f * dq[i] * dt; norm += q_eb[i] * q_eb[i]; }
        for (unsigned i = 0U; i < 4U; ++i) q_eb[i] /= sqrtf(norm);

        if ((float)n * dt > settle_s)
        {
            worst_late_tilt = fmaxf(worst_late_tilt, out.tilt_deg);
            worst_late_roll = fmaxf(worst_late_roll, fabsf(out.body_rate_dps[2]));
            if (hold_roll) worst_late_roll = fmaxf(worst_late_roll, fabsf(out.error_deg[2]));
        }
    }
    printf("  mismatch x%.1f%s: tilt after %.1f s <= %.3f deg, roll rate/angle <= %.3f, peak deflection %.2f deg\n",
           (double)mismatch, hold_roll ? " roll-hold" : "", (double)settle_s, (double)worst_late_tilt, (double)worst_late_roll, (double)peak_defl);
    assert(worst_late_tilt < 0.5f && worst_late_roll < 2.0f && peak_defl <= c.max_deflection_deg + 1e-3f);
}

/** @brief Run all canard tests. @return Process status. */
int main(void)
{
    allocation_cases();
    gate_cases();
    roll_only_cases();
    soft_start_case();
    recovery_case();
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float s = sqrtf(0.5f);
    const float y_is_nose[4] = {s, s, 0.0f, 0.0f}; /* Sensor +Y along the nose. */
    puts("Canard flight: 10 deg tilt + 90 dps roll at 100 m/s, servo lag 20 ms, no fin stability");
    flight_case(1.0f, identity, 2.0f, false);
    flight_case(0.5f, identity, 3.0f, false);
    flight_case(2.0f, identity, 3.0f, false);
    flight_case(1.0f, y_is_nose, 2.0f, false);
    flight_case(1.0f, identity, 3.0f, true);
    puts("Canard closed loop: exact/half/double effectiveness, rotated IMU and roll-angle hold PASS");
    return 0;
}
