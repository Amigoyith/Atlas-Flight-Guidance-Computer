/**
 * @file atlas_flight.c
 * @brief Flight-phase state machine from ADXL375 magnitude and MS5611 altitude.
 *
 * Major functions:
 * - AtlasFlight_Step(): dwell-confirmed boost and burnout from |a|, then a barometric
 *   descent dwell (or timeout backstop) ends the coast. Phases never go backwards.
 */
#include "atlas_flight.h"
#include <math.h>
#include <string.h>

/** @brief Fill defaults. @param c Result. */
void AtlasFlight_DefaultConfig(AtlasFlightConfig *c)
{
    memset(c, 0, sizeof(*c));
    c->boost_accel_g = 5.0f;
    c->boost_dwell_ms = 100U;
    c->coast_accel_g = 1.0f;
    c->coast_dwell_ms = 250U;
    c->descent_rate_m_s = 1.0f;
    c->descent_dwell_ms = 10000U;
    c->coast_timeout_ms = 60000U;
    c->adxl_max_age_ms = 100U;
    c->baro_max_age_ms = 500U;
    c->ground_tau_s = 5.0f;
    c->baro_alpha = 0.3f;
    c->baro_beta = 0.05f;
}

/** @brief Start in PAD. @param f State. @param c Configuration. @param now_ms Tick.
 * @return False if the configuration is unusable. */
bool AtlasFlight_Init(AtlasFlight *f, const AtlasFlightConfig *c, uint32_t now_ms)
{
    memset(f, 0, sizeof(*f));
    f->phase = ATLAS_FLIGHT_PAD;
    f->phase_since_ms = now_ms;
    if (c == NULL || !(c->boost_accel_g > c->coast_accel_g) || !(c->coast_accel_g > 0.0f) ||
        !(c->descent_rate_m_s >= 0.0f) || c->descent_dwell_ms == 0U || c->adxl_max_age_ms == 0U ||
        c->baro_max_age_ms == 0U || !(c->ground_tau_s > 0.0f) ||
        !(c->baro_alpha > 0.0f && c->baro_alpha <= 1.0f) || !(c->baro_beta >= 0.0f && c->baro_beta < 1.0f))
    {
        f->config.boost_accel_g = INFINITY; /* Never leaves PAD. */
        return false;
    }
    f->config = *c;
    return true;
}

/** @brief Enter a new phase. @param f State. @param p Phase. @param r Reason. @param now Tick. */
static void flight_enter(AtlasFlight *f, AtlasFlightPhase p, AtlasFlightReason r, uint32_t now)
{
    f->phase = p;
    f->reason = r;
    f->phase_since_ms = now;
    f->dwell_active = false;
    f->descending = false;
    if (p == ATLAS_FLIGHT_ENGINE_ON) f->launch_ms = now;
}

/** @brief Pressure to altitude above the pad (international standard atmosphere).
 * @param pa Pressure. @param ground Pad pressure. @return Metres. */
static float flight_altitude(float pa, float ground)
{
    return 44330.0f * (1.0f - powf(pa / ground, 0.190295f));
}

/** @brief Consume a new barometer sample: pad reference, then an alpha-beta tracker.
 * @param f State. @param in Inputs. */
static void flight_baro(AtlasFlight *f, const AtlasFlightInput *in)
{
    const AtlasFlightConfig *c = &f->config;
    const float pa = (float)in->pressure_pa;
    const uint32_t dt_ms = in->baro_ms - f->last_baro_ms;
    if (!(pa > 1000.0f && pa < 120000.0f)) return; /* Implausible: ignore. */
    if (!f->have_baro)
    {
        f->ground_pa = pa;
        f->have_baro = true;
        f->last_baro_ms = in->baro_ms;
        return;
    }
    f->last_baro_ms = in->baro_ms;
    const float dt = (float)dt_ms * 0.001f;
    if (!(dt > 0.0f) || dt > 2.0f) { f->baro_filter_ready = false; return; }
    /* The pad reference follows slow weather/temperature drift until launch. */
    if (f->phase == ATLAS_FLIGHT_PAD)
        f->ground_pa += (pa - f->ground_pa) * (dt / (c->ground_tau_s + dt));
    const float z = flight_altitude(pa, f->ground_pa);
    if (!isfinite(z)) return;
    if (!f->baro_filter_ready)
    {
        f->altitude_m = z;
        f->climb_m_s = 0.0f;
        f->baro_filter_ready = true;
    }
    else
    {
        const float predicted = f->altitude_m + f->climb_m_s * dt;
        const float residual = z - predicted;
        f->altitude_m = predicted + c->baro_alpha * residual;
        f->climb_m_s += (c->baro_beta / dt) * residual;
    }
    if (f->altitude_m > f->max_altitude_m) f->max_altitude_m = f->altitude_m;
}

/** @brief Advance on new samples. @param f State. @param in Inputs. @return Phase after the step. */
AtlasFlightPhase AtlasFlight_Step(AtlasFlight *f, const AtlasFlightInput *in)
{
    const AtlasFlightConfig *c = &f->config;
    const uint32_t now = in->now_ms;

    if (in->baro_valid && (!f->have_baro || in->baro_ms != f->last_baro_ms) &&
        (uint32_t)(now - in->baro_ms) <= c->baro_max_age_ms)
        flight_baro(f, in);

    /* Acceleration dwell: PAD looks for boost, ENGINE_ON for burnout. */
    const bool adxl_fresh = in->adxl_valid && (uint32_t)(now - in->adxl_ms) <= c->adxl_max_age_ms;
    if (!adxl_fresh) f->dwell_active = false;
    else if (!f->have_adxl || in->adxl_ms != f->last_adxl_ms)
    {
        f->have_adxl = true;
        f->last_adxl_ms = in->adxl_ms;
        const float a = sqrtf(in->adxl_g[0] * in->adxl_g[0] + in->adxl_g[1] * in->adxl_g[1] +
                              in->adxl_g[2] * in->adxl_g[2]);
        f->accel_g = a;
        bool candidate = false;
        uint32_t dwell = 0U;
        if (f->phase == ATLAS_FLIGHT_PAD) { candidate = a > c->boost_accel_g; dwell = c->boost_dwell_ms; }
        else if (f->phase == ATLAS_FLIGHT_ENGINE_ON) { candidate = a < c->coast_accel_g; dwell = c->coast_dwell_ms; }
        if (!candidate || !isfinite(a)) f->dwell_active = false;
        else
        {
            if (!f->dwell_active) { f->dwell_active = true; f->dwell_since_ms = in->adxl_ms; }
            if ((uint32_t)(in->adxl_ms - f->dwell_since_ms) >= dwell)
            {
                if (f->phase == ATLAS_FLIGHT_PAD)
                    flight_enter(f, ATLAS_FLIGHT_ENGINE_ON, ATLAS_FLIGHT_REASON_BOOST_ACCEL, now);
                else
                    flight_enter(f, ATLAS_FLIGHT_COAST, ATLAS_FLIGHT_REASON_BURNOUT_ACCEL, now);
                return f->phase;
            }
        }
    }

    if (f->phase == ATLAS_FLIGHT_COAST)
    {
        /* Descent: the filtered altitude must keep decreasing, on fresh samples, for the whole dwell. */
        const bool baro_fresh = f->baro_filter_ready && (uint32_t)(now - f->last_baro_ms) <= c->baro_max_age_ms;
        if (baro_fresh && f->climb_m_s < -c->descent_rate_m_s)
        {
            if (!f->descending) { f->descending = true; f->descent_since_ms = f->last_baro_ms; }
            if ((uint32_t)(now - f->descent_since_ms) >= c->descent_dwell_ms)
            {
                flight_enter(f, ATLAS_FLIGHT_CHUTE, ATLAS_FLIGHT_REASON_BARO_DESCENT, now);
                return f->phase;
            }
        }
        else f->descending = false;
        if (c->coast_timeout_ms != 0U && (uint32_t)(now - f->phase_since_ms) >= c->coast_timeout_ms)
            flight_enter(f, ATLAS_FLIGHT_CHUTE, ATLAS_FLIGHT_REASON_COAST_TIMEOUT, now);
    }
    return f->phase;
}

/** @brief Short phase name. @param p Phase. @return Constant string. */
const char *AtlasFlight_PhaseName(AtlasFlightPhase p)
{
    switch (p)
    {
        case ATLAS_FLIGHT_PAD: return "PAD";
        case ATLAS_FLIGHT_ENGINE_ON: return "ENGINE_ON";
        case ATLAS_FLIGHT_COAST: return "COAST";
        case ATLAS_FLIGHT_CHUTE: return "CHUTE";
        default: return "UNKNOWN";
    }
}
