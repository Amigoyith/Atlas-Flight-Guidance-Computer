/**
 * @file atlas_clock_map.h
 * @brief Map a sensor's free-running timestamp counter onto the MCU microsecond clock.
 *
 * Major functions:
 * - AtlasClockMap_Init(): starts from the data-sheet tick period.
 * - AtlasClockMap_AddPair(): accepts a (sensor ticks, MCU us) pair read back to back.
 * - AtlasClockMap_ToMcuUs(): converts any nearby sensor timestamp to MCU time.
 *
 * The sensor oscillator is trimmed only to about a percent. The tick period is
 * therefore re-measured against the crystal-derived MCU counter over baselines of
 * at least ATLAS_CLOCK_MAP_BASELINE_US, and every pair re-anchors the offset so
 * conversion error does not accumulate. Pure arithmetic; host testable.
 */

#ifndef ATLAS_CLOCK_MAP_H
#define ATLAS_CLOCK_MAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Minimum MCU interval between tick-period measurements. */
#define ATLAS_CLOCK_MAP_BASELINE_US       (1000000UL)
/** Accepted deviation of a measured tick period from nominal, in parts per thousand. */
#define ATLAS_CLOCK_MAP_PERIOD_LIMIT_PPT  (50U)
/** A pair farther than this from prediction is treated as a sensor-clock discontinuity. */
#define ATLAS_CLOCK_MAP_RESYNC_LIMIT_US   (2000U)

/** @brief Conversion state. */
typedef struct
{
    float nominal_us_per_tick;
    float us_per_tick;        /**< Current (measured once available) tick period. */
    uint32_t anchor_ticks;    /**< Most recent accepted pair. */
    uint32_t anchor_us;
    uint32_t baseline_ticks;  /**< Start of the current period measurement. */
    uint32_t baseline_us;
    bool anchored;
    bool period_measured;
    uint32_t pairs;           /**< Pairs accepted. */
    uint32_t period_updates;  /**< Successful period measurements. */
    uint32_t resyncs;         /**< Discontinuities or out-of-range periods. */
} AtlasClockMap;

/**
 * @brief Reset the map to the nominal tick period with no anchor.
 * @param map Map to initialize.
 * @param nominal_us_per_tick Data-sheet tick period; must be positive.
 */
void AtlasClockMap_Init(AtlasClockMap *map, float nominal_us_per_tick);

/**
 * @brief Add a simultaneous observation of both clocks.
 * @param map Initialized map.
 * @param sensor_ticks Sensor counter value.
 * @param mcu_us MCU microsecond time at which that counter value was current.
 */
void AtlasClockMap_AddPair(AtlasClockMap *map, uint32_t sensor_ticks, uint32_t mcu_us);

/**
 * @brief Convert a sensor timestamp to MCU microseconds.
 * @param map Anchored map.
 * @param sensor_ticks Sensor counter value within about +/-2^31 ticks of the anchor.
 * @param mcu_us Destination MCU time.
 * @return false until the first pair has been added.
 */
bool AtlasClockMap_ToMcuUs(const AtlasClockMap *map, uint32_t sensor_ticks, uint32_t *mcu_us);

#ifdef __cplusplus
}
#endif

#endif /* ATLAS_CLOCK_MAP_H */
