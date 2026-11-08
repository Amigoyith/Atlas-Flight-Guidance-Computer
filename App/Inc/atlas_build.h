/**
 * @file atlas_build.h
 * @brief Explicit build-profile boundary; diagnostics must never become flight mode.
 * Major functions and definitions: ATLAS_BRINGUP selects the non-actuating bench image.
 * Define it through the Bringup CMake preset, not by editing this default.
 */
#ifndef ATLAS_BUILD_H
#define ATLAS_BUILD_H
#ifndef ATLAS_BRINGUP
#define ATLAS_BRINGUP 0
#endif
#if ATLAS_BRINGUP != 0 && ATLAS_BRINGUP != 1
#error "ATLAS_BRINGUP must be exactly 0 or 1"
#endif
#ifndef ATLAS_SERVO_BENCH
#define ATLAS_SERVO_BENCH 0
#endif
#if (ATLAS_SERVO_BENCH != 0 && ATLAS_SERVO_BENCH != 1) || (ATLAS_SERVO_BENCH && !ATLAS_BRINGUP)
#error "ServoBench requires ATLAS_BRINGUP=1 and ATLAS_SERVO_BENCH=1"
#endif
#if ATLAS_SERVO_BENCH
#define ATLAS_BENCH_PROFILE "servo_bench"
#else
#define ATLAS_BENCH_PROFILE "bringup"
#endif
#define ATLAS_BRINGUP_VERSION "1.5.0"
/** @brief BNO085 use (CMake option ATLAS_USE_BNO085, default OFF). Off, it is never probed,
 * sampled or required, so its absence cannot inhibit outputs or stop the watchdog refresh. */
#ifndef ATLAS_USE_BNO085
#define ATLAS_USE_BNO085 0
#endif
#endif
