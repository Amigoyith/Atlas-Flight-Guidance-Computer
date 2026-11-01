/**
 * @file atlas_canard_fusion.c
 * @brief Adapter from AtlasStabilization (Fusion AHRS, ENU) to AtlasCanardInput.
 *
 * Major functions:
 * - AtlasCanard_InputFromEstimator(): copies Fusion's quaternion and derives
 *   linear (sensor axes) and earth (ENU) acceleration from the raw sample.
 */
#include "atlas_canard_fusion.h"
#include <math.h>
#include <string.h>

/** @brief Fill a canard input from the estimator after it consumed this sample.
 * @param est Estimator, already updated with sample (Fusion must be ENU).
 * @param sample The IMU sample just given to AtlasStabilization_Update().
 * @param bias Calibration whose gyro bias the estimator used, or NULL.
 * @param airspeed_m_s Measured airspeed, or negative for the controller's estimate.
 * @param enable Flight-phase and safety permission for the canards.
 * @param in Result.
 * @return False if the estimator has no attitude, the sample is invalid or Fusion is not ENU. */
bool AtlasCanard_InputFromEstimator(const AtlasStabilization *est, const AtlasStabilizationSample *sample,
                                    const AtlasStabilizationConfig *bias, float airspeed_m_s, bool enable,
                                    AtlasCanardInput *in)
{
    if (est == NULL || sample == NULL || in == NULL) return false;
    memset(in, 0, sizeof(*in));
    /* FusionAhrs members are documented as private; the convention is read only to
       refuse a frame the controller does not use (it needs Z = up and ENU headings). */
    if (!est->initialized || !sample->valid || est->ahrs.convention != FusionConventionEnu) return false;

    /* Fusion keeps |q| only approximately 1 (fast inverse square root); normalize so a
       5 g boost is not mis-scaled. Its third matrix row is "up" in sensor axes. */
    const FusionQuaternion raw = FusionAhrsGetQuaternion(&est->ahrs);
    const float norm = FusionQuaternionNorm(raw);
    if (!isfinite(norm) || norm < 0.5f) return false;
    const FusionQuaternion q = FusionQuaternionScale(raw, 1.0f / norm);
    const FusionMatrix r = FusionQuaternionToMatrix(q);
    const FusionVector accel = {.array = {sample->accel_g[0], sample->accel_g[1], sample->accel_g[2]}};
    const FusionVector up_sensor = {.array = {r.element.zx, r.element.zy, r.element.zz}};
    const FusionVector linear = FusionVectorSubtract(accel, up_sensor);
    FusionVector earth = FusionMatrixMultiply(r, accel);
    earth.axis.z -= 1.0f; /* Remove gravity: at rest the accelerometer reads +1 g up. */

    memcpy(in->quaternion, q.array, sizeof(in->quaternion));
    memcpy(in->linear_accel_g, linear.array, sizeof(in->linear_accel_g));
    memcpy(in->earth_accel_g, earth.array, sizeof(in->earth_accel_g));
    for (unsigned i = 0U; i < 3U; ++i)
        in->gyro_dps[i] = sample->gyro_dps[i] - (bias != NULL ? bias->bias_dps[i] : 0.0f);
    in->airspeed_m_s = airspeed_m_s;
    in->timestamp_us = sample->timestamp_ms * 1000U; /* Differences stay exact across wrap. */
    in->enable = enable;
    return true;
}
