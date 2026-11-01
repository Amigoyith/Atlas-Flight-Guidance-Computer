/**
 * @file test_canard_fusion.c
 * @brief Full chain: Fusion stabilization estimator -> adapter -> canard controller -> rocket.
 *
 * Major functions:
 * - adapter_cases(): quaternion copy, gravity removal before and after launch, rejects.
 * - chain_flight(): settle on the pad, latch launch, then fly gyro-only through the real
 *   estimator with a pitch/roll disturbance; the true attitude must return upright.
 */
#include "atlas_canard_fusion.h"
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

/** @brief Rotate v by q or its inverse. @param q Unit quaternion. @param inverse Conjugate.
 * @param v Input. @param o Result. */
static void qrot(const float q[4], int inverse, const float v[3], float o[3])
{
    const float c[4] = {q[0], inverse ? -q[1] : q[1], inverse ? -q[2] : q[2], inverse ? -q[3] : q[3]};
    const float p[4] = {0.0f, v[0], v[1], v[2]}, ci[4] = {c[0], -c[1], -c[2], -c[3]};
    float t[4], r[4];
    qmul(c, p, t);
    qmul(t, ci, r);
    o[0] = r[1]; o[1] = r[2]; o[2] = r[3];
}

/** @brief Feed one sample to the estimator. @param est Estimator. @param s Sample. */
static void feed(AtlasStabilization *est, const AtlasStabilizationSample *s)
{
    AtlasStabilization_Update(est, s, NULL, s->timestamp_ms);
}

/** @brief Adapter: attitude copy, gravity removal, launch coasting and rejections. */
static void adapter_cases(void)
{
    AtlasStabilization est;
    memset(&est, 0, sizeof(est));
    AtlasStabilizationSample s = {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 1000U, true};
    AtlasCanardInput in;
    assert(!AtlasCanard_InputFromEstimator(&est, &s, NULL, 100.0f, true, &in)); /* no attitude yet */
    for (unsigned i = 0U; i < 60U; ++i) { s.timestamp_ms += 5U; feed(&est, &s); }
    assert(est.ready);
    assert(AtlasCanard_InputFromEstimator(&est, &s, NULL, 100.0f, true, &in));
    const FusionQuaternion q = FusionAhrsGetQuaternion(&est.ahrs);
    const float qn = FusionQuaternionNorm(q);
    for (unsigned i = 0U; i < 4U; ++i) assert(fabsf(in.quaternion[i] - q.array[i] / qn) < 1e-6f);
    for (unsigned i = 0U; i < 3U; ++i) assert(fabsf(in.earth_accel_g[i]) < 1e-3f && fabsf(in.linear_accel_g[i]) < 1e-3f);
    assert(in.timestamp_us == s.timestamp_ms * 1000U && in.enable && in.airspeed_m_s == 100.0f);

    /* Boost along the nose: estimator ignores the accelerometer, the adapter still sees +4 g. */
    s.accel_g[2] = 5.0f;
    for (unsigned i = 0U; i < 4U; ++i) { s.timestamp_ms += 5U; feed(&est, &s); }
    assert(est.launched);
    AtlasStabilizationConfig bias;
    memset(&bias, 0, sizeof(bias));
    bias.bias_dps[2] = 2.0f;
    s.gyro_dps[2] = 2.0f;
    assert(AtlasCanard_InputFromEstimator(&est, &s, &bias, -1.0f, false, &in));
    assert(fabsf(in.earth_accel_g[2] - 4.0f) < 1e-3f && fabsf(in.linear_accel_g[2] - 4.0f) < 1e-3f);
    assert(fabsf(in.gyro_dps[2]) < 1e-6f && !in.enable);
    const FusionVector broken = FusionAhrsGetEarthAcceleration(&est.ahrs);
    assert(broken.axis.z < -0.9f); /* Why the adapter does not use Fusion's getter after launch. */
    s.valid = false;
    assert(!AtlasCanard_InputFromEstimator(&est, &s, NULL, 100.0f, true, &in));
    puts("Canard adapter: Fusion quaternion, gravity removal at rest and in boost (gyro-only), bias, rejects PASS");
}

/** @brief Pad, launch, then disturbed coast flown through the real estimator. */
static void chain_flight(void)
{
    AtlasCanardConfig c;
    AtlasCanard ctl;
    AtlasCanardOutput out;
    AtlasCanardInput in;
    AtlasCanard_DefaultConfig(&c);
    assert(AtlasCanard_Init(&ctl, &c));

    AtlasStabilization est;
    memset(&est, 0, sizeof(est));
    AtlasStabilizationSample s = {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 1000U, true};
    for (unsigned i = 0U; i < 100U; ++i) { s.timestamp_ms += 4U; feed(&est, &s); }
    assert(est.ready && AtlasStabilization_Launch(&est));

    /* Truth: body = sensor (identity mount), upright, kicked into pitch, yaw and roll. */
    float q_true[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float omega[3] = {RAD(30.0f), RAD(-20.0f), RAD(90.0f)}, servo[ATLAS_CANARD_MAX] = {0};
    const float dt = 0.004f, tau = 0.02f, nose_b[3] = {0.0f, 0.0f, 1.0f};
    float peak_tilt = 0.0f, late_tilt = 0.0f, late_roll = 0.0f, worst_est = 0.0f;
    for (unsigned n = 0U; n < 1250U; ++n) /* 5 s */
    {
        /* Coast: drag decelerates along the nose; gravity is not felt by the accelerometer. */
        const float drag_body[3] = {0.0f, 0.0f, -0.3f};
        memcpy(s.accel_g, drag_body, sizeof(s.accel_g));
        for (unsigned i = 0U; i < 3U; ++i) s.gyro_dps[i] = DEG(omega[i]);
        s.timestamp_ms += 4U;
        feed(&est, &s);
        assert(AtlasCanard_InputFromEstimator(&est, &s, NULL, 100.0f, true, &in));
        AtlasCanard_Step(&ctl, &in, &out);

        float nose_true[3], nose_est[3];
        qrot(q_true, 0, nose_b, nose_true);
        qrot(in.quaternion, 0, nose_b, nose_est);
        const float tilt_true = DEG(acosf(fminf(1.0f, nose_true[2])));
        const float est_err = DEG(acosf(fminf(1.0f, nose_true[0]*nose_est[0] + nose_true[1]*nose_est[1] +
                                                     nose_true[2]*nose_est[2])));
        peak_tilt = fmaxf(peak_tilt, tilt_true);
        worst_est = fmaxf(worst_est, est_err);
        if (n >= 750U) { late_tilt = fmaxf(late_tilt, tilt_true); late_roll = fmaxf(late_roll, fabsf(DEG(omega[2]))); }

        /* Plant: servo lag, canard moments, rigid-body rotation. */
        const float k = out.dynamic_pressure_pa * c.canard_area_m2 * c.canard_lift_slope_per_rad;
        float m[3] = {0.0f, 0.0f, 0.0f};
        for (unsigned j = 0U; j < c.canard_count; ++j)
        {
            servo[j] += (out.deflection_deg[j] - servo[j]) * dt / tau;
            const float phi = RAD(c.canard[j].azimuth_deg), d = RAD(servo[j]);
            m[0] += k * d * -c.canard_arm_m * cosf(phi);
            m[1] += k * d * -c.canard_arm_m * sinf(phi);
            m[2] += k * d * c.canard_radius_m;
        }
        const float *I = c.inertia_kg_m2;
        const float gyro_term[3] = {(I[2] - I[1]) * omega[1] * omega[2], (I[0] - I[2]) * omega[2] * omega[0],
                                    (I[1] - I[0]) * omega[0] * omega[1]};
        for (unsigned i = 0U; i < 3U; ++i) omega[i] += (m[i] - gyro_term[i]) / I[i] * dt;
        const float w[4] = {0.0f, omega[0], omega[1], omega[2]};
        float dq[4], norm = 0.0f;
        qmul(q_true, w, dq);
        for (unsigned i = 0U; i < 4U; ++i) { q_true[i] += 0.5f * dq[i] * dt; norm += q_true[i] * q_true[i]; }
        for (unsigned i = 0U; i < 4U; ++i) q_true[i] /= sqrtf(norm);
    }
    printf("  peak true tilt %.2f deg, after 3 s: tilt <= %.3f deg, roll rate <= %.3f dps; estimator nose error <= %.3f deg\n",
           (double)peak_tilt, (double)late_tilt, (double)late_roll, (double)worst_est);
    assert(peak_tilt > 1.0f && late_tilt < 0.5f && late_roll < 2.0f && worst_est < 0.5f);
    puts("Canard chain: Fusion estimator (ENU, gyro-only after launch) + canard PD returns the true attitude upright PASS");
}

/** @brief Run the adapter and full-chain tests. @return Process status. */
int main(void)
{
    adapter_cases();
    chain_flight();
    return 0;
}
