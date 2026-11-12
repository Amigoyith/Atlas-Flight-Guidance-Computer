/**
 * @file atlas_flight.h
 * @brief Flight-phase state machine: PAD -> ENGINE_ON -> COAST -> CHUTE -> LANDED, one way only.
 *
 * Phase TRANSITIONS use only the ADXL375 (high-g accelerometer) and the MS5611
 * barometer. They are deliberately independent of STATE ESTIMATION (Fusion on the
 * LSM6DSV16B and MMC5983MA), which tells the controller where the rocket points.
 *
 * - PAD -> ENGINE_ON: ADXL magnitude above boost_accel_g for boost_dwell_ms.
 * - ENGINE_ON -> COAST: ADXL magnitude below coast_accel_g for coast_dwell_ms.
 * - COAST -> CHUTE: barometric altitude decreasing for descent_dwell_ms (10 s), or the
 *   coast_timeout_ms backstop. Canard control is allowed only in COAST.
 * - CHUTE -> LANDED: barometric climb rate within +-landed_rate_m_s for landed_dwell_ms
 *   (10 s), or the landed_timeout_ms backstop. LANDED only starts the buzzer heartbeat.
 * - Apogee is an event, not a phase: apogee_ms is the time of the highest altitude after launch.
 *
 * Major functions: DefaultConfig, Init, Step. Pure C: no HAL, RTOS or allocation.
 */
#ifndef ATLAS_FLIGHT_H
#define ATLAS_FLIGHT_H
#include <stdbool.h>
#include <stdint.h>

/** @brief Flight phase. Values are stable: they are logged and sent by telemetry. */
typedef enum
{
    ATLAS_FLIGHT_PAD = 0,       /**< On the pad; estimator calibrates; canards off. */
    ATLAS_FLIGHT_ENGINE_ON = 1, /**< Motor burning; estimator gyro + magnetometer; canards off. */
    ATLAS_FLIGHT_COAST = 2,     /**< Upward coast; canard roll control on. */
    ATLAS_FLIGHT_CHUTE = 3,     /**< Descent; canards neutral, then signal off. */
    ATLAS_FLIGHT_LANDED = 4     /**< On the ground after descent; buzzer heartbeat. */
} AtlasFlightPhase;

/** @brief Why the last transition happened. */
typedef enum
{
    ATLAS_FLIGHT_REASON_NONE = 0,
    ATLAS_FLIGHT_REASON_BOOST_ACCEL,   /**< ADXL above boost_accel_g for the dwell. */
    ATLAS_FLIGHT_REASON_BURNOUT_ACCEL, /**< ADXL below coast_accel_g for the dwell. */
    ATLAS_FLIGHT_REASON_BARO_DESCENT,  /**< Altitude decreasing for descent_dwell_ms. */
    ATLAS_FLIGHT_REASON_COAST_TIMEOUT, /**< Backstop: coast lasted coast_timeout_ms. */
    ATLAS_FLIGHT_REASON_LANDED_STILL,  /**< Altitude steady for landed_dwell_ms. */
    ATLAS_FLIGHT_REASON_LANDED_TIMEOUT /**< Backstop: descent lasted landed_timeout_ms. */
} AtlasFlightReason;

/** @brief Thresholds. All times in milliseconds; set them before Init. */
typedef struct
{
    float boost_accel_g;        /**< PAD -> ENGINE_ON above this |a| (default 5 g). */
    uint32_t boost_dwell_ms;    /**< ...held this long (default 100 ms). */
    float coast_accel_g;        /**< ENGINE_ON -> COAST below this |a| (default 1 g). */
    uint32_t coast_dwell_ms;    /**< ...held this long (default 250 ms). */
    float descent_rate_m_s;     /**< "Decreasing" means filtered climb rate below -this (default 1 m/s). */
    uint32_t descent_dwell_ms;  /**< COAST -> CHUTE after decreasing this long (default 10 s). */
    uint32_t coast_timeout_ms;  /**< COAST -> CHUTE backstop if the barometer never confirms (default 60 s; 0 = off). */
    float landed_rate_m_s;      /**< "Steady" means |filtered climb rate| below this (default 1 m/s). */
    uint32_t landed_dwell_ms;   /**< CHUTE -> LANDED after steady this long (default 10 s). */
    uint32_t landed_timeout_ms; /**< CHUTE -> LANDED backstop (default 15 min in CHUTE; 0 = off). */
    uint32_t adxl_max_age_ms;   /**< An older ADXL sample breaks a dwell (default 100 ms). */
    uint32_t baro_max_age_ms;   /**< An older barometer sample breaks a descent (default 500 ms). */
    float ground_tau_s;         /**< Pad pressure reference averaging time constant (default 5 s). */
    float baro_alpha, baro_beta; /**< Alpha-beta altitude/rate tracker gains (default 0.3, 0.05). */
} AtlasFlightConfig;

/** @brief Sensor inputs for one step. Only new samples (changed timestamp) are used. */
typedef struct
{
    float adxl_g[3];          /**< ADXL375 acceleration, any axes (only |a| is used). */
    uint32_t adxl_ms;         /**< ADXL sample tick. */
    bool adxl_valid;
    int32_t pressure_pa;      /**< MS5611 compensated pressure. */
    uint32_t baro_ms;         /**< Barometer sample tick. */
    bool baro_valid;
    uint32_t now_ms;          /**< Current tick. */
} AtlasFlightInput;

/** @brief State-machine state; also the telemetry/log view. */
typedef struct
{
    AtlasFlightConfig config;
    AtlasFlightPhase phase;
    AtlasFlightReason reason;
    uint32_t phase_since_ms;       /**< Tick the current phase began. */
    uint32_t launch_ms;            /**< Tick ENGINE_ON began (0 before launch). */
    uint32_t dwell_since_ms;       /**< Start of the current boost/burnout candidate. */
    uint32_t descent_since_ms;     /**< Start of the current descent (or, in CHUTE, steady) candidate. */
    uint32_t apogee_ms;            /**< Tick of the highest altitude after launch (0 before launch). */
    uint32_t last_adxl_ms, last_baro_ms;
    float accel_g;                 /**< Latest ADXL magnitude. */
    float ground_pa;               /**< Pad reference pressure (frozen at launch). */
    float altitude_m;              /**< Filtered altitude above the pad. */
    float climb_m_s;               /**< Filtered climb rate (positive up). */
    float max_altitude_m;          /**< Highest filtered altitude seen. */
    bool dwell_active, descending, have_adxl, have_baro, baro_filter_ready;
} AtlasFlight;

/** @brief Fill the defaults listed in AtlasFlightConfig. @param c Result. */
void AtlasFlight_DefaultConfig(AtlasFlightConfig *c);
/** @brief Start in PAD. @param f State. @param c Configuration (copied). @param now_ms Current tick.
 * @return False on an invalid configuration (state left in PAD, never advances). */
bool AtlasFlight_Init(AtlasFlight *f, const AtlasFlightConfig *c, uint32_t now_ms);
/** @brief Advance on new samples. Transitions are one way and at most one per call.
 * @param f State. @param in Inputs. @return The phase after this step. */
AtlasFlightPhase AtlasFlight_Step(AtlasFlight *f, const AtlasFlightInput *in);
/** @brief Stable short name for logs. @param p Phase. @return Constant string. */
const char *AtlasFlight_PhaseName(AtlasFlightPhase p);

#endif
