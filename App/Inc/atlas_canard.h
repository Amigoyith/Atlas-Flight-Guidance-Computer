/**
 * @file atlas_canard.h
 * @brief Canard attitude controller (PID) driven by Fusion AHRS outputs in ENU.
 *
 * Major functions:
 * - AtlasCanard_DefaultConfig(): placeholder geometry and starting gains.
 * - AtlasCanard_Init(): validates the configuration and builds the canard allocation.
 * - AtlasCanard_Reset(): clears launch detection, speed estimate and integrators (on the pad).
 * - AtlasCanard_SetTarget()/CaptureTarget(): attitude to hold (default: nose straight up).
 * - AtlasCanard_Step(): one control update -> canard deflections and servo pulses.
 * - AtlasCanard_RollOnlyConfig(): roll-rate damping that holds wherever the spin stops.
 * - AtlasCanard_MountFromAxes(): IMU mounting from the nose and azimuth-0 directions.
 *
 * Frames. Earth is ENU (configure Fusion with FusionConventionEnu). The body frame
 * has +Z along the nose (roll axis) and +X toward canard azimuth 0; +Y completes a
 * right-handed set. With the default mounting, the IMU sensor axes are the body
 * axes, so the identity attitude means "nose up".
 *
 * Control law. Gains are angular-acceleration gains, independent of the airframe:
 *   alpha = Kp * error + Ki * integral(error) - Kd * body_rate
 * (natural frequency sqrt(Kp) rad/s, damping ratio Kd / (2 sqrt(Kp))). The moment
 * inertia * alpha is then allocated to the canards using their geometry and the
 * current dynamic pressure, so new canard dimensions need no gain retuning.
 *
 * Pure computation: no HAL, RTOS, allocation or time source. One owner per state.
 */
#ifndef ATLAS_CANARD_H
#define ATLAS_CANARD_H
#include <stdbool.h>
#include <stdint.h>

#define ATLAS_CANARD_MAX 6U
/** Hard ceiling on the servo slew limit; configurations above it are rejected. */
#define ATLAS_CANARD_MAX_SLEW_DPS 600.0f
/** Longest allowed soft-start ramp. */
#define ATLAS_CANARD_MAX_SOFT_START_US 10000000U
/** Longest allowed neutral-signal period after parachute deployment. */
#define ATLAS_CANARD_MAX_RECOVERY_SIGNAL_US 120000000U

/** @brief One canard: where it sits around the body and how its servo is driven. */
typedef struct
{
    float azimuth_deg;      /**< Angle of the canard span from body +X toward +Y. */
    float servo_center_us;  /**< Pulse at zero deflection (servo trim). */
    float servo_us_per_deg; /**< Pulse change per degree; the sign sets direction. */
} AtlasCanardFin;

/** @brief Airframe, servo and tuning parameters. Replace placeholders when known. */
typedef struct
{
    float sensor_from_body[4];      /**< w,x,y,z: rotates body vectors into IMU axes. */
    uint32_t canard_count;          /**< 3..ATLAS_CANARD_MAX; pitch, yaw and roll need >= 3. */
    AtlasCanardFin canard[ATLAS_CANARD_MAX];
    float canard_area_m2;           /**< Planform area of ONE canard. */
    float canard_lift_slope_per_rad;/**< CL_alpha of one canard (per radian of deflection). */
    float canard_arm_m;             /**< Canard centre of pressure ahead of the CG along +Z. */
    float canard_radius_m;          /**< Body axis to canard centre of pressure. */
    float inertia_kg_m2[3];         /**< Ixx, Iyy (pitch/yaw), Izz (roll) about the CG. */
    float air_density_kg_m3;        /**< Constant density used for dynamic pressure. */
    float min_dynamic_pressure_pa;  /**< Below this the canards are held neutral. */
    float kp_tilt, kd_tilt, ki_tilt;/**< Pitch/yaw gains: 1/s^2, 1/s, 1/s^3. */
    float kp_roll, kd_roll, ki_roll;/**< Roll gains; kp_roll = 0 gives roll-rate damping only. */
    float integral_limit_rad_s;     /**< Clamp on each integrated error (rad*s). */
    float max_tilt_error_deg;       /**< Larger pointing errors are treated as this size. */
    float max_deflection_deg;       /**< Mechanical deflection limit of every canard. */
    float max_rate_dps;             /**< Servo slew cap, 0 < cap <= ATLAS_CANARD_MAX_SLEW_DPS. */
    float min_pulse_us, max_pulse_us;
    float launch_accel_g;           /**< Linear-acceleration magnitude that indicates boost. */
    uint32_t launch_hold_us;        /**< ...sustained for this long latches "launched". */
    /** Roll hold-on-stop: damp roll rate only, then hold the angle where the spin stopped.
     * Holding starts after |roll rate| < enter for settle time and ends above exit; while
     * holding, kp_roll pulls back to the latched angle. Off: kp_roll targets the target roll. */
    bool roll_hold_on_stop;
    float roll_hold_enter_dps, roll_hold_exit_dps;
    uint32_t roll_hold_settle_us;
    /** Soft start: on every transition into ACTIVE, command authority ramps linearly from
     * 0 to 1 over this time (0 = off, at most ATLAS_CANARD_MAX_SOFT_START_US). */
    uint32_t soft_start_us;
    /** After parachute deployment the canards go neutral at once; this long afterwards
     * signal_off latches and the caller must stop sending PWM (default 20 s). */
    uint32_t recovery_signal_off_us;
} AtlasCanardConfig;

/** @brief One control-rate sample. Fusion arrays copy directly (w,x,y,z and x,y,z). */
typedef struct
{
    float quaternion[4];      /**< FusionAhrsGetQuaternion(): sensor -> ENU. */
    float earth_accel_g[3];   /**< FusionAhrsGetEarthAcceleration(): ENU, gravity removed. */
    float linear_accel_g[3];  /**< FusionAhrsGetLinearAcceleration(): sensor axes. */
    float gyro_dps[3];        /**< Gyroscope sample given to FusionAhrsUpdate*(), sensor axes. */
    float airspeed_m_s;       /**< Measured airspeed, or negative to use the internal estimate. */
    uint32_t timestamp_us;    /**< Sample time; wraps safely. */
    bool enable;              /**< Caller's flight-phase and safety permission. */
    bool parachute_deployed;  /**< Recovery event seen; latched by the controller until Reset. */
} AtlasCanardInput;

/** @brief Why the canards are, or are not, being driven. */
typedef enum
{
    ATLAS_CANARD_ACTIVE = 0,
    ATLAS_CANARD_DISABLED,       /**< enable was false. */
    ATLAS_CANARD_LOW_PRESSURE,   /**< Dynamic pressure below min_dynamic_pressure_pa. */
    ATLAS_CANARD_INVALID_INPUT,  /**< Non-finite or non-unit input; neutral commanded. */
    ATLAS_CANARD_NOT_CONFIGURED, /**< AtlasCanard_Init() has not succeeded. */
    ATLAS_CANARD_RECOVERY        /**< Parachute deployed: neutral, then signal off. */
} AtlasCanardState;

/** @brief Controller state; treat as private. */
typedef struct
{
    AtlasCanardConfig config;
    float allocation[ATLAS_CANARD_MAX][3]; /**< deflection(rad) per unit moment / q*S*CLa. */
    float target[4];                       /**< Desired body -> ENU attitude. */
    float integral[3];
    float velocity_m_s[3];
    float deflection_rad[ATLAS_CANARD_MAX];
    uint32_t last_us, launch_since_us, roll_still_since_us, active_since_us, deployed_since_us;
    float roll_hold_twist;                 /**< Latched roll angle (rad) relative to target. */
    bool configured, have_time, launched, launch_pending, saturated;
    bool roll_settling, roll_holding, was_active, deployed, signal_off;
} AtlasCanard;

/** @brief Result of one step, for servos and telemetry. */
typedef struct
{
    AtlasCanardState state;
    float deflection_deg[ATLAS_CANARD_MAX];
    uint16_t pulse_us[ATLAS_CANARD_MAX];
    float tilt_deg;            /**< Angle between the nose and the target nose direction. */
    float error_deg[3];        /**< Body-axis errors: pitch/yaw (X, Y) and roll (Z). */
    float body_rate_dps[3];
    float moment_nm[3];        /**< Commanded moment before deflection limits. */
    float dynamic_pressure_pa;
    float speed_m_s;
    bool launched, saturated;
    bool roll_holding;         /**< Roll hold-on-stop has latched an angle. */
    float authority;           /**< Soft-start command scale, 0..1 (1 once ramped). */
    bool signal_off;           /**< Recovery period over: stop PWM to every canard servo. */
} AtlasCanardOutput;

/** @brief Fill placeholder geometry (4 canards in a + layout) and starting gains.
 * @param c Destination. */
void AtlasCanard_DefaultConfig(AtlasCanardConfig *c);
/** @brief Roll-only test setup: no pitch/yaw command, roll rate damped to zero, then the
 * angle where it stopped is held. Starts from AtlasCanard_DefaultConfig().
 * @param c Destination. */
void AtlasCanard_RollOnlyConfig(AtlasCanardConfig *c);
/** @brief Build sensor_from_body from two directions measured in IMU axes.
 * @param nose_sensor Body +Z (nose) in sensor axes. @param x_hint_sensor Direction of canard
 * azimuth 0; only its part perpendicular to the nose is used.
 * @param sensor_from_body Result quaternion (w,x,y,z).
 * @return False if either direction is unusable or they are nearly parallel. */
bool AtlasCanard_MountFromAxes(const float nose_sensor[3], const float x_hint_sensor[3],
                               float sensor_from_body[4]);
/** @brief Validate a configuration and prepare the controller; target = nose up.
 * @param s State. @param c Configuration (copied).
 * @return False if any value is invalid or the canards cannot produce all three moments. */
bool AtlasCanard_Init(AtlasCanard *s, const AtlasCanardConfig *c);
/** @brief Forget launch, speed, integrators and deflection history. @param s State. */
void AtlasCanard_Reset(AtlasCanard *s);
/** @brief Hold a given body attitude. @param s State.
 * @param body_to_enu Desired attitude, w,x,y,z (normalized here). @return False if invalid. */
bool AtlasCanard_SetTarget(AtlasCanard *s, const float body_to_enu[4]);
/** @brief Hold the attitude in this sample, e.g. on the launch rail.
 * @param s State. @param in Current sample. @return False if the sample is invalid. */
bool AtlasCanard_CaptureTarget(AtlasCanard *s, const AtlasCanardInput *in);
/** @brief Run one control update. Neutral (zero deflection, slew-limited) unless active.
 * @param s State. @param in Sample. @param out Result.
 * @return True when the canards are actively controlling. */
bool AtlasCanard_Step(AtlasCanard *s, const AtlasCanardInput *in, AtlasCanardOutput *out);
#endif
