# 2026-10-25 changes (scratch notes — delete after review)

Branch: `mohan_fusion_xio` (from `main`). Nothing is committed. Fusion is staged as a submodule; everything else is unstaged.

## 1. Fusion AHRS added as a submodule

- `ThirdParty/Fusion` → https://github.com/xioTechnologies/Fusion, pinned at `a8d7224` (MIT). `.gitmodules` added.
- Only `FusionAhrs.c` is compiled. Fusion's bias estimator and compass helper are not used.
- Clone with `--recurse-submodules`, or run `git submodule update --init`.

## 2. Estimator now runs on Fusion

[App/Src/atlas_stabilization.c](../App/Src/atlas_stabilization.c) and [App/Inc/atlas_stabilization.h](../App/Inc/atlas_stabilization.h):

- The hand-written gyro rotation plus accelerometer blend is replaced by `FusionAhrsUpdate`, with a zero magnetometer vector whenever the magnetometer is off or unusable (see 4b/4c).
- `AtlasStabilization` gained a `FusionAhrs ahrs` member. Public API, freshness checks, calibration and horn geometry are unchanged. `atlas_io.c` needed no call-site changes.
- Start: the quaternion is set from the measured up vector (exact, including the 180° case) and Fusion's startup ramp is skipped.
- Each update sets the sample period from the measured interval, so 240 Hz jitter is handled.
- Settings: 240 Hz nominal, gain 2.0 (about a 0.5 s accelerometer time constant), acceleration rejection 10° with a 5 s timeout.
- The 0.85–1.15 g accelerometer window is Atlas's own gate, kept from the old code. It is separate from Fusion's direction-based rejection.

## 3. Launch latch (found already in the working tree; tested)

- 3 consecutive samples at 3 g or more (`ATLAS_STAB_LAUNCH_G`, `ATLAS_STAB_LAUNCH_SAMPLES`) set `launched`.
- After launch the accelerometer is passed to Fusion as a zero vector, so attitude is gyro-only. Sample gaps, invalid samples and rate checks no longer restart the estimator.
- Cleared only by zeroing the state. `AtlasStabilization_Launch()` fails if there is no attitude yet.

## 4. Fusion flags exposed

- `ATLAS_STAB_FLAG_*` bits: `LAUNCHED`, `STARTUP`, `ACCEL_IGNORED`, `ACCEL_RECOVERY`, `OVERRANGE_RECOVERY`.
- `AtlasStabilization` has `flags`, `accel_error_deg` and `accel_trigger` (0.0–1.0, recovery at 1.0). They come from `FusionAhrsGetFlags` and `FusionAhrsGetInternalStates` and are refreshed on every AHRS step.
- Copied into `AtlasStabilizationSnapshot` in [atlas_io.c](../App/Src/atlas_io.c).
- Bring-up JSON ([atlas_bringup.c](../App/Src/atlas_bringup.c)): `ahrs_flags`, `accel_err_cdeg`, `accel_trigger_pm`. [tools/bringup/protocol.py](../tools/bringup/protocol.py) range-checks them and accepts frames without them.
- The dashboard does not display these yet.

## 4b. Magnetometer added (heading)

- `AtlasStabilization_SetMagnetometer()` stores the newest MMC5983MA reading (kept separate from `AtlasStabilizationSample` so existing initializers are untouched). `AtlasIo_StabilizationMagnetometer()` carries it from the sensor owner ([atlas_bringup.c](../App/Src/atlas_bringup.c), 10 Hz) to the estimator.
- Each IMU step passes Fusion the field only if it is fresh (within 250 ms), finite, and 0.2–0.8 gauss after hard-iron offset and axis mapping. Otherwise Fusion gets a zero vector.
- Fusion rejection: 20° for the field (looser than the 10° for tilt), same 5 s timeout. If a field sample is present at start, Fusion's startup ramp is kept so heading locks in about 3 s instead of waiting for the rejection timeout.
- Pre-launch only, like the accelerometer. After launch heading coasts on the gyro.
- Output: `heading_deg` (clockwise from magnetic north, 0-360) of IMU axis `ATLAS_STAB_HEADING_AXIS` (2 = +Z, the body +X face normal), computed from Fusion's quaternion. No heading is reported when that axis is within about 12° of vertical. Magnetic north only; no declination.
- New flags: `MAG_IGNORED` (bit 5), `MAG_RECOVERY` (6), `HEADING_VALID` (7). New state: `mag_error_deg`, `mag_trigger`, `mag_locked`. JSON: `mag_err_cdeg`, `mag_trigger_pm`, `heading_cdeg` (protocol.py allows `ahrs_flags` up to 255).
- Fusion's magnetometer feedback is perpendicular to gravity, so it corrects heading only; the test confirms the up vector does not move.
- **UNVERIFIED constants** in [atlas_stabilization.h](../App/Inc/atlas_stabilization.h): `ATLAS_STAB_MAG_TO_IMU` (identity) and `ATLAS_STAB_MAG_OFFSET_GAUSS` (zero). The MMC5983MA-to-LSM6DSV16B axis relationship is not documented in the repo. Heading is wrong until these are checked against the schematic and calibrated on the vehicle (hard-iron especially near servos and wiring).
- Test `magnetometer`: 5 headings within 1° against an independent NWU construction, gravity unmoved, wrong-magnitude and stale fields rejected, rejection then recovery on a 90° disagreement, heading frozen after launch with a flipped field, vertical axis has no heading. Swapping two axes in the map makes it fail.

## 4c. Magnetometer build switch

- CMake option `ATLAS_STAB_USE_MAGNETOMETER`, default OFF, passed as a compile definition; the header also defaults it to 0, so IAR and ad-hoc builds are OFF. The gate is the first line of `stab_mag`, so OFF behaves exactly like an absent field. Up is unaffected either way.
- Tests: `magnetometer` has two forms. OFF (default build) asserts the field is ignored, no lock, startup skipped. ON (`-DATLAS_STAB_USE_MAGNETOMETER=1`) runs the full heading test. `run_bringup_tests.ps1` builds both.
- [docs/STABILIZATION.md](STABILIZATION.md) estimation section rewritten for Fusion, launch and the switch.
- Not done: the CMake presets do not list the option (they list `ATLAS_SERVO_BENCH` explicitly); the default is OFF anyway. IAR does not define it.

## 5. Build and test wiring

- [CMakeLists.txt](../CMakeLists.txt): `FusionAhrs.c` and the Fusion include path.
- [EWARM/Atlas.ewp](../EWARM/Atlas.ewp): include path and a "Fusion AHRS" file group.
- `Tests/bringup/run_bringup_tests.ps1`, `Tests/host/run_tests.ps1`, `Tests/services/run_service_tests.ps1`, `Tests/review/run_review_probes.ps1`: Fusion include path added, plus `FusionAhrs.c` wherever `atlas_stabilization.c` is linked.
- [docs/reference/PROVENANCE.md](reference/PROVENANCE.md): Fusion row, license and submodule note.

## 6. Tests added in `Tests/bringup/test_stabilization.c`

- `fusion_attitude`: exact start over 234 orientations; gyro-only rotation at jittered 240 Hz matches an independent Rodrigues calculation; flag checks (accelerometer ignored on a 40° disagreement, trigger rises, `ACCEL_RECOVERY` fires at 1.0).
- `launch_coast`: launch latch, accelerometer ignored during boost, gap and invalid samples, plus the `LAUNCHED` and `ACCEL_IGNORED` flags.
- `magnetometer`: two forms, selected by `ATLAS_STAB_USE_MAGNETOMETER` (see 4b and 4c).

## Verified (macOS, gcc/clang)

Passing: stabilization unit tests, the servo-bench and bring-up IO tests, the storage-owner test, the stabilization and servo station Python tests, and the stabilization UI test. The IO tests were built with `-D"__attribute__(x)="` on the command line, because Apple's compiler rejects the `.atlas_dma` section attribute.

## Not verified

- No ARM cross-compile or CMake build (`arm-none-eabi-gcc` and `cmake` are not installed here).
- The `.ps1` scripts were not run (no PowerShell).
- `atlas_bringup.c` (new JSON fields and the magnetometer hand-off) needs the STM32 HAL and was not compiled; reviewed by eye only.
- Magnetometer axis alignment and hard-iron calibration are unverified (see 4b).
- No hardware testing.

## Open items

- **The flight firmware does not run the estimator.** `AtlasIo_StabilizationSample` is called only from the bring-up sensor loop under `ATLAS_SERVO_BENCH`; `atlas_rtos.c` never feeds it. Nothing in a normal build knows "up" yet. Wiring the RTOS snapshot into `AtlasStabilization_Update` is the next step.
- Post-launch the estimate is gyro-only and drifts. The LSM6DSV16B gyro range and the 3 g launch threshold are untuned; the 500 dps rate check is bypassed after launch.
- Fusion's 10° acceleration rejection is still on before launch, so a bump on the pad can make it ignore the accelerometer for up to 5 s. Disable it pre-launch if the accelerometer should always be trusted until launch.
- The dashboard (`tools/bringup/web/stabilization.js`) does not show the new flag or heading fields.
- With uncorrected gyro bias, a steady bias of 3 dps gives about 1.5° of steady tilt error. Commissioned bias removes it.
