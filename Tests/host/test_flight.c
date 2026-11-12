/**
 * @file test_flight.c
 * @brief Host tests for the flight-phase state machine (ADXL375 + barometer only).
 * Major functions: main runs dwell, false-trigger, descent, landing, timeout and config cases.
 */
#include "atlas_flight.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, msg); ++failures; } } while (0)

/** @brief Pressure at an altitude above a 101325 Pa pad (ISA). @param h Metres. @return Pa. */
static int32_t pressure_at(float h) { return (int32_t)lroundf(101325.0f * powf(1.0f - h / 44330.0f, 5.255f)); }

/** @brief Simple simulated sensor feed at the 100 Hz application rate. */
typedef struct { AtlasFlight f; uint32_t t; float altitude; } Sim;

/** @brief Advance 10 ms with a given ADXL magnitude (along Z) and altitude.
 * @param s Simulation. @param g ADXL reading. @param h Altitude for the 10 Hz barometer. @param baro Baro available. */
static AtlasFlightPhase sim_step(Sim *s, float g, float h, bool baro)
{
    s->t += 10U;
    AtlasFlightInput in = {0};
    in.adxl_g[2] = g; in.adxl_ms = s->t; in.adxl_valid = true;
    in.pressure_pa = pressure_at(h); in.baro_ms = s->t - (s->t % 100U); in.baro_valid = baro;
    in.now_ms = s->t;
    return AtlasFlight_Step(&s->f, &in);
}

/** @brief Run for a duration at constant inputs. @return Final phase. */
static AtlasFlightPhase sim_run(Sim *s, uint32_t ms, float g, float h, bool baro)
{
    AtlasFlightPhase p = s->f.phase;
    for (uint32_t i = 0; i < ms / 10U; ++i) p = sim_step(s, g, h, baro);
    return p;
}

/** @brief Start a simulation on the pad with 2 s of quiet data. */
static void sim_init(Sim *s)
{
    AtlasFlightConfig c;
    AtlasFlight_DefaultConfig(&c);
    s->t = 1000U;
    CHECK(AtlasFlight_Init(&s->f, &c, s->t), "default config valid");
    sim_run(s, 2000U, 1.0f, 0.0f, true);
}

/** @brief Boost needs > 5 g held for 100 ms; short spikes and handling bumps do not count. */
static void boost_cases(void)
{
    Sim s;
    sim_init(&s);
    CHECK(s.f.phase == ATLAS_FLIGHT_PAD, "starts on the pad");
    sim_run(&s, 80U, 8.0f, 0.0f, true);   /* 80 ms bump: too short */
    sim_run(&s, 500U, 1.0f, 0.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_PAD, "80 ms spike does not launch");
    sim_run(&s, 1000U, 4.9f, 0.0f, true); /* below threshold */
    CHECK(s.f.phase == ATLAS_FLIGHT_PAD, "4.9 g does not launch");
    uint32_t start = s.t, at = 0U;
    for (int i = 0; i < 30 && s.f.phase == ATLAS_FLIGHT_PAD; ++i) { sim_step(&s, 6.0f, 0.0f, true); at = s.t; }
    CHECK(s.f.phase == ATLAS_FLIGHT_ENGINE_ON, "6 g held launches");
    CHECK(at - start >= 100U && at - start <= 120U, "launch after the 100 ms dwell");
    CHECK(s.f.reason == ATLAS_FLIGHT_REASON_BOOST_ACCEL, "boost reason");
}

/** @brief Burnout needs < 1 g held for 250 ms; a brief dip during boost does not count. */
static void burnout_cases(void)
{
    Sim s;
    sim_init(&s);
    sim_run(&s, 200U, 8.0f, 0.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_ENGINE_ON, "in boost");
    sim_run(&s, 200U, 0.5f, 50.0f, true); /* 200 ms dip */
    sim_run(&s, 500U, 8.0f, 100.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_ENGINE_ON, "200 ms dip is not burnout");
    sim_run(&s, 2000U, 1.2f, 300.0f, true); /* drag still above 1 g */
    CHECK(s.f.phase == ATLAS_FLIGHT_ENGINE_ON, "1.2 g drag keeps ENGINE_ON");
    uint32_t start = s.t, at = 0U;
    for (int i = 0; i < 50 && s.f.phase == ATLAS_FLIGHT_ENGINE_ON; ++i) { sim_step(&s, 0.6f, 400.0f, true); at = s.t; }
    CHECK(s.f.phase == ATLAS_FLIGHT_COAST, "below 1 g held -> COAST");
    CHECK(at - start >= 250U && at - start <= 270U, "coast after the 250 ms dwell");
}

/** @brief Ballistic coast to apogee then descent: CHUTE exactly ~10 s after descent starts,
 * not during ascent, with barometer noise. */
static void descent_cases(void)
{
    Sim s;
    sim_init(&s);
    sim_run(&s, 200U, 8.0f, 0.0f, true);
    sim_run(&s, 300U, 0.5f, 0.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_COAST, "coasting");
    srand(7);
    /* Coast: v0 = 200 m/s upward, gravity only -> apogee at ~20.4 s, ~2040 m. */
    float h = 300.0f, v = 200.0f;
    uint32_t apogee_ms = 0U, chute_ms = 0U, descent_start_ms = 0U;
    for (int i = 0; i < 6000 && s.f.phase == ATLAS_FLIGHT_COAST; ++i)
    {
        const float prev_v = v;
        v -= 9.81f * 0.01f;
        if (v < -30.0f) v = -30.0f; /* drogue terminal velocity */
        h += v * 0.01f;
        if (prev_v > 0.0f && v <= 0.0f) apogee_ms = s.t;
        if (descent_start_ms == 0U && v < -1.0f) descent_start_ms = s.t;
        const float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * 1.0f; /* +-0.5 m */
        sim_step(&s, 0.3f, h + noise, true);
        if (s.f.phase == ATLAS_FLIGHT_CHUTE) chute_ms = s.t;
    }
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "descent -> CHUTE");
    CHECK(s.f.reason == ATLAS_FLIGHT_REASON_BARO_DESCENT, "baro descent reason");
    CHECK(chute_ms > apogee_ms + 10000U, "CHUTE only after 10 s past apogee");
    CHECK(chute_ms < descent_start_ms + 11500U, "CHUTE within ~1.5 s of the 10 s descent dwell");
    printf("  apogee at %.1f s (recorded %.1f s), falling > 1 m/s at %.1f s, CHUTE at %.1f s (sim time)\n",
           apogee_ms * 0.001f, s.f.apogee_ms * 0.001f, descent_start_ms * 0.001f, chute_ms * 0.001f);
    CHECK(s.f.apogee_ms + 1500U > apogee_ms && s.f.apogee_ms < apogee_ms + 1500U, "apogee time recorded within 1.5 s");
    CHECK(fabsf(s.f.max_altitude_m - h) > 100.0f && s.f.max_altitude_m > 2000.0f, "peak altitude kept after apogee");
    /* One way: CHUTE is final even if inputs look like boost again. */
    sim_run(&s, 1000U, 9.0f, 0.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "phases never go backwards");
}

/** @brief A brief descent (e.g. transonic baro error) shorter than 10 s does not end the coast. */
static void false_descent_case(void)
{
    Sim s;
    sim_init(&s);
    sim_run(&s, 200U, 8.0f, 0.0f, true);
    sim_run(&s, 300U, 0.5f, 0.0f, true);
    float h = 500.0f;
    for (int i = 0; i < 500; ++i) { h -= 0.3f; sim_step(&s, 0.5f, h, true); } /* 5 s apparent fall */
    for (int i = 0; i < 300; ++i) { h += 1.0f; sim_step(&s, 0.5f, h, true); }  /* climbing again */
    CHECK(s.f.phase == ATLAS_FLIGHT_COAST, "5 s apparent descent does not deploy");
}

/** @brief No barometer at all: the coast timeout backstop still ends control. */
static void timeout_case(void)
{
    Sim s;
    sim_init(&s);
    sim_run(&s, 200U, 8.0f, 0.0f, true);
    sim_run(&s, 300U, 0.5f, 0.0f, true);
    sim_run(&s, 59000U, 0.5f, 0.0f, false);
    CHECK(s.f.phase == ATLAS_FLIGHT_COAST, "still coasting before the timeout");
    sim_run(&s, 1500U, 0.5f, 0.0f, false);
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE && s.f.reason == ATLAS_FLIGHT_REASON_COAST_TIMEOUT, "timeout backstop");
}

/** @brief Reach CHUTE quickly: boost, burnout, then 11 s falling at 6 m/s from 500 m.
 * @param s Simulation. @return Altitude at CHUTE. */
static float sim_to_chute(Sim *s)
{
    sim_init(s);
    sim_run(s, 200U, 8.0f, 0.0f, true);
    sim_run(s, 300U, 0.5f, 0.0f, true);
    float h = 500.0f;
    for (int i = 0; i < 1500 && s->f.phase == ATLAS_FLIGHT_COAST; ++i) { h -= 0.06f; sim_step(s, 1.0f, h, true); }
    return h;
}

/** @brief Under the parachute (6 m/s, noisy) it stays in CHUTE; once on the ground and steady
 * for 10 s it is LANDED, and LANDED is final. */
static void landed_cases(void)
{
    Sim s;
    float h = sim_to_chute(&s);
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "in CHUTE");
    srand(11);
    while (h > 0.0f && s.f.phase == ATLAS_FLIGHT_CHUTE)
    {
        h -= 0.06f;
        const float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * 1.0f;
        sim_step(&s, 1.0f, (h > 0.0f ? h : 0.0f) + noise, true);
    }
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "descending at 6 m/s is not landed");
    const uint32_t touchdown = s.t;
    uint32_t landed_ms = 0U;
    for (int i = 0; i < 3000 && s.f.phase == ATLAS_FLIGHT_CHUTE; ++i)
    {
        const float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * 1.0f;
        sim_step(&s, 1.0f, noise, true);
        landed_ms = s.t;
    }
    CHECK(s.f.phase == ATLAS_FLIGHT_LANDED && s.f.reason == ATLAS_FLIGHT_REASON_LANDED_STILL, "steady on ground -> LANDED");
    CHECK(landed_ms >= touchdown + 10000U && landed_ms <= touchdown + 13000U, "LANDED 10-13 s after touchdown");
    printf("  touchdown at %.1f s, LANDED at %.1f s (sim time)\n", touchdown * 0.001f, landed_ms * 0.001f);
    sim_run(&s, 1000U, 9.0f, 300.0f, true);
    CHECK(s.f.phase == ATLAS_FLIGHT_LANDED, "LANDED is final");

    /* A short pause in the descent (e.g. main deployment, 5 s) does not count as landed. */
    h = sim_to_chute(&s);
    sim_run(&s, 5000U, 1.0f, h, true);
    for (int i = 0; i < 500; ++i) { h -= 0.06f; sim_step(&s, 1.0f, h, true); }
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "5 s steady in the air is not landed");

    /* No barometer under the parachute: the 15 min backstop still reaches LANDED. */
    sim_to_chute(&s);
    sim_run(&s, 890000U, 1.0f, 0.0f, false);
    CHECK(s.f.phase == ATLAS_FLIGHT_CHUTE, "still CHUTE before the landed timeout");
    sim_run(&s, 11000U, 1.0f, 0.0f, false);
    CHECK(s.f.phase == ATLAS_FLIGHT_LANDED && s.f.reason == ATLAS_FLIGHT_REASON_LANDED_TIMEOUT, "landed timeout backstop");
}

/** @brief Stale or invalid ADXL samples break a dwell instead of extending it. */
static void stale_case(void)
{
    Sim s;
    sim_init(&s);
    sim_run(&s, 60U, 8.0f, 0.0f, true);
    for (int i = 0; i < 30; ++i) /* ADXL frozen at an old 8 g sample for 300 ms */
    {
        s.t += 10U;
        AtlasFlightInput in = {0};
        in.adxl_g[2] = 8.0f; in.adxl_ms = 2050U; in.adxl_valid = true; in.now_ms = s.t;
        AtlasFlight_Step(&s.f, &in);
    }
    CHECK(s.f.phase == ATLAS_FLIGHT_PAD, "stale high-g sample does not launch");
}

/** @brief Invalid configuration never leaves PAD. */
static void config_case(void)
{
    AtlasFlightConfig c;
    AtlasFlight_DefaultConfig(&c);
    c.coast_accel_g = 6.0f; /* above boost threshold */
    AtlasFlight f;
    CHECK(!AtlasFlight_Init(&f, &c, 0U), "coast >= boost rejected");
    AtlasFlightInput in = {0};
    for (uint32_t t = 10U; t < 2000U; t += 10U)
    { in.adxl_g[2] = 20.0f; in.adxl_ms = t; in.adxl_valid = true; in.now_ms = t; AtlasFlight_Step(&f, &in); }
    CHECK(f.phase == ATLAS_FLIGHT_PAD, "invalid config stays on the pad");
}

/** @brief Run all cases. @return Process status. */
int main(void)
{
    boost_cases();
    burnout_cases();
    descent_cases();
    false_descent_case();
    timeout_case();
    landed_cases();
    stale_case();
    config_case();
    if (failures) { printf("%d flight FSM check(s) failed\n", failures); return 1; }
    printf("Flight FSM tests passed\n");
    return 0;
}
