/**
 * @file atlas_canard_fusion.h
 * @brief Builds canard controller input from the Fusion-based stabilization estimator.
 *
 * Major functions:
 * - AtlasCanard_InputFromEstimator(): quaternion, gravity-removed accelerations and
 *   bias-corrected rates for AtlasCanard_Step(), from AtlasStabilization (Fusion, ENU).
 *
 * Accelerations are computed from the raw accelerometer sample and Fusion's attitude,
 * not from FusionAhrsGetLinear/EarthAcceleration(): after launch the estimator feeds
 * Fusion a zero accelerometer vector (gyro-only attitude), which makes those getters
 * report -1 g instead of the real acceleration.
 */
#ifndef ATLAS_CANARD_FUSION_H
#define ATLAS_CANARD_FUSION_H
#include <stdbool.h>
#include "atlas_canard.h"
#include "atlas_stabilization.h"

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
                                    AtlasCanardInput *in);
#endif
