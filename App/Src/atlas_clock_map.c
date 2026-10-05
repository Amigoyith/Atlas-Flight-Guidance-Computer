/**
 * @file atlas_clock_map.c
 * @brief Sensor-tick to MCU-microsecond conversion with on-line period calibration.
 *
 * Major functions:
 * - AtlasClockMap_AddPair(): re-anchors, detects discontinuities, measures the period.
 * - AtlasClockMap_ToMcuUs(): wrap-safe signed extrapolation from the latest anchor.
 */

#include "atlas_clock_map.h"

#include <stddef.h>

/**
 * @brief Round a float to the nearest int32 without libm.
 * @param value Value well inside the int32 range.
 * @return Rounded value.
 */
static int32_t atlas_clock_round(float value)
{
    return (int32_t)((value >= 0.0F) ? (value + 0.5F) : (value - 0.5F));
}

/**
 * @brief Reset to nominal with no anchor.
 * @param map Map to initialize.
 * @param nominal_us_per_tick Data-sheet tick period.
 */
void AtlasClockMap_Init(AtlasClockMap *map, float nominal_us_per_tick)
{
    if ((map == NULL) || !(nominal_us_per_tick > 0.0F))
    {
        return;
    }
    map->nominal_us_per_tick = nominal_us_per_tick;
    map->us_per_tick = nominal_us_per_tick;
    map->anchor_ticks = 0U;
    map->anchor_us = 0U;
    map->baseline_ticks = 0U;
    map->baseline_us = 0U;
    map->anchored = false;
    map->period_measured = false;
    map->pairs = 0U;
    map->period_updates = 0U;
    map->resyncs = 0U;
}

/**
 * @brief Accept one simultaneous observation of both clocks.
 * @param map Initialized map.
 * @param sensor_ticks Sensor counter value.
 * @param mcu_us MCU time for that counter value.
 */
void AtlasClockMap_AddPair(AtlasClockMap *map, uint32_t sensor_ticks, uint32_t mcu_us)
{
    uint32_t predicted_us = 0U;

    if ((map == NULL) || !(map->nominal_us_per_tick > 0.0F))
    {
        return;
    }
    ++map->pairs;
    if (!map->anchored)
    {
        map->anchored = true;
        map->anchor_ticks = map->baseline_ticks = sensor_ticks;
        map->anchor_us = map->baseline_us = mcu_us;
        return;
    }

    /* A sensor reset or a stale/corrupt read shows up as a large prediction error. */
    (void)AtlasClockMap_ToMcuUs(map, sensor_ticks, &predicted_us);
    {
        const int32_t error_us = (int32_t)(mcu_us - predicted_us);
        const uint32_t since_anchor_us = mcu_us - map->anchor_us;
        const uint32_t allowed_us = ATLAS_CLOCK_MAP_RESYNC_LIMIT_US +
            (uint32_t)(((uint64_t)since_anchor_us * ATLAS_CLOCK_MAP_PERIOD_LIMIT_PPT) / 1000U);
        if ((error_us > (int32_t)allowed_us) || (error_us < -(int32_t)allowed_us))
        {
            ++map->resyncs;
            map->anchor_ticks = map->baseline_ticks = sensor_ticks;
            map->anchor_us = map->baseline_us = mcu_us;
            return;
        }
    }

    map->anchor_ticks = sensor_ticks;
    map->anchor_us = mcu_us;

    {
        const uint32_t elapsed_us = mcu_us - map->baseline_us;
        const uint32_t elapsed_ticks = sensor_ticks - map->baseline_ticks;
        if ((elapsed_us < ATLAS_CLOCK_MAP_BASELINE_US) || (elapsed_ticks == 0U))
        {
            return;
        }
        {
            const float measured = (float)elapsed_us / (float)elapsed_ticks;
            const float limit = map->nominal_us_per_tick *
                                ((float)ATLAS_CLOCK_MAP_PERIOD_LIMIT_PPT / 1000.0F);
            if ((measured > (map->nominal_us_per_tick + limit)) ||
                (measured < (map->nominal_us_per_tick - limit)))
            {
                ++map->resyncs;
            }
            else
            {
                /* One-second pair jitter is ~1e-5; light smoothing tracks thermal drift. */
                map->us_per_tick = map->period_measured ?
                    (map->us_per_tick + (0.5F * (measured - map->us_per_tick))) : measured;
                map->period_measured = true;
                ++map->period_updates;
            }
        }
        map->baseline_ticks = sensor_ticks;
        map->baseline_us = mcu_us;
    }
}

/**
 * @brief Convert a sensor timestamp to MCU microseconds.
 * @param map Anchored map.
 * @param sensor_ticks Sensor counter value near the anchor.
 * @param mcu_us Destination.
 * @return true when anchored.
 */
bool AtlasClockMap_ToMcuUs(const AtlasClockMap *map, uint32_t sensor_ticks, uint32_t *mcu_us)
{
    if ((map == NULL) || (mcu_us == NULL) || !map->anchored)
    {
        return false;
    }
    {
        const int32_t delta_ticks = (int32_t)(sensor_ticks - map->anchor_ticks);
        *mcu_us = map->anchor_us +
                  (uint32_t)atlas_clock_round((float)delta_ticks * map->us_per_tick);
    }
    return true;
}
