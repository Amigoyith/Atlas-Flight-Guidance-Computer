# Canard control

[`atlas_canard.c`](../App/Src/atlas_canard.c) is a PID attitude controller for canards. It takes the outputs of the [Fusion AHRS](https://github.com/xioTechnologies/Fusion) and returns a deflection and a servo pulse for each canard. It is a pure calculation module. In the flight image, [`atlas_flight_app.c`](../App/Src/atlas_flight_app.c) runs it during upward coast only, fed by the flight estimator; PWM output stays off until the vehicle's servo limits are qualified. See [Flight state machine](FLIGHT_STATE_MACHINE.md).

## What it does

1. **Attitude error** from the Fusion quaternion, against a target attitude. The default target is nose straight up; `AtlasCanard_CaptureTarget()` instead holds the rail attitude.
   - **Pitch/yaw:** the angle between the nose and the target nose direction, about body X and Y. It is clamped to `max_tilt_error_deg`.
   - **Roll:** the twist about the nose. It is used only if `kp_roll > 0`; otherwise roll *rate* is damped to zero.
2. **PID** per axis, in angular-acceleration units: `alpha = Kp*error + Ki*integral - Kd*body_rate`. The rate comes from the gyro, not from differentiating the error.
3. **Moment** = inertia × alpha.
4. **Allocation:** the canard geometry and the current dynamic pressure `q = ½ρv²` turn the moment into a deflection per canard (least squares over all canards).
5. **Limits:**
   - All deflections are scaled down together to `max_deflection_deg`, so the pitch/yaw/roll mix is kept.
   - Optional soft start (`soft_start_us`): after every engage, authority ramps from 0 to 1.
   - Each servo is slew-limited to `max_rate_dps`, which can never exceed `ATLAS_CANARD_MAX_SLEW_DPS` (600 °/s).
   - Pulses are clamped to `min_pulse_us`/`max_pulse_us`.
6. **Gates:** the canards return to neutral (through the slew limit) when `enable` is false, when `q < min_dynamic_pressure_pa`, or when the input is invalid.
7. **Parachute cutoff:** once `parachute_deployed` is seen, the canards go neutral at once and stay neutral regardless of the other inputs. 20 s later (`recovery_signal_off_us`), `signal_off` latches, and the caller must then stop sending PWM to every canard servo. Both stay latched until `AtlasCanard_Reset()`, even if the deployment input clears.
   - Who sets `parachute_deployed` is the flight integration's job. Candidates are a recovery-event input from the commercial altimeters, or Atlas apogee/descent detection.
   - The bench roll test has no parachute and never sets it.

Because the gains act on angular acceleration, they don't depend on the airframe. When the canard size or position changes, update the geometry fields; the response stays the same.

## Servo interface: open loop, no feedback

The KST X10 V8.0 servos have three wires only: signal (orange), supply (red) and ground (brown). Their position potentiometer is internal and is **not reported back**. Atlas only sends a PWM pulse; it never knows where a canard actually is.
- `deflection_deg` and `pulse_us` in the output are **commands**, not measured angles. The slew limiter tracks the last command, not the shaft.
- The only feedback in the loop is the **IMU**: if a canard lags, stalls under air load, or is linked wrong, the controller sees it only as the rocket rolling differently than expected, and keeps correcting from the gyro.
- A stalled, disconnected or reversed canard therefore cannot be detected directly. The bench sign check and a pre-flight horn-position check are the only protection against it.
- After `signal_off` the servo stops receiving pulses. It may hold or release its last position. The 20 s neutral period before cutoff exists so that the last commanded position is neutral.

Signal levels, from the [KST X10 V8.0 specification](https://www.kst-servo-shop.de/media/44/69/96/1749814229/KST_0106_X10_V8_Datenblatt_04_2025_de.pdf?ts=1751463833) (2025-04):

| Item | Datasheet value | Atlas |
|---|---|---|
| Signal HIGH | min 3.3 V, max 5.0 V | Driven directly by STM32 timer pins at 3.3 V: **exactly at the minimum, no margin** |
| Signal LOW | 0.0–1.5 V | ≈ 0 V |
| Pulse range | 900–2100 µs | Clamped to `min_pulse_us`/`max_pulse_us` |
| Position mapping | 1000/1500/2000 µs = −50°/0°/+50° (10 µs/deg) | `servo_us_per_deg` = ±10 |
| Frame | 1520 µs / 333 Hz | 333 Hz |
| Speed (no load) | 0.16 / 0.14 / 0.12 / 0.10 s per 60° at 4.8 / 6.0 / 7.4 / 8.4 V (375–600 °/s) | Slew cap 200 °/s, below the slowest rating |
| Torque | 6.2 / 7.5 / 9.5 / 10.8 kgf·cm at 4.8 / 6.0 / 7.4 / 8.4 V | Must exceed 2× the hinge moment |

The 3.3 V HIGH level has no margin: any droop in the MCU supply puts the signal below the servo's minimum. Before flight, scope the signal at the servo connector with the servo attached. If HIGH reads below 3.3 V, add a buffer that drives 3.3–5.0 V (for example a 74AHCT-family gate supplied from 5 V).

## Feeding it from Fusion

Fusion ([xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion), submodule `ThirdParty/Fusion`) runs inside the stabilization estimator ([`atlas_stabilization.c`](../App/Src/atlas_stabilization.c)), configured for **ENU**. Before launch, the estimator fuses gyro and accelerometer (plus the magnetometer, if enabled). From launch onward it is gyro-only. [`atlas_canard_fusion.c`](../App/Src/atlas_canard_fusion.c) turns its state into controller input:

```c
AtlasStabilization_Update(&est, &sample, &calibration, now_ms);   /* Fusion step */
AtlasCanardInput in;
if (AtlasCanard_InputFromEstimator(&est, &sample, &calibration,
                                   -1.0f,                    /* or a measured airspeed */
                                   flight_phase_allows_canards, &in))
    AtlasCanard_Step(&canard, &in, &out);                   /* out.pulse_us[] -> servos */
```

The adapter does three things:
- It normalizes Fusion's quaternion. Fusion keeps the length only approximately 1, and the error would mis-scale a 5 g boost.
- It subtracts the calibrated gyro bias.
- It computes linear and earth acceleration from the **raw** accelerometer sample. It cannot use Fusion's `FusionAhrsGetEarthAcceleration()`: after launch the estimator gives Fusion a zero accelerometer vector, so that getter reports −1 g during flight.

It refuses an estimator that has no attitude yet, an invalid sample, or a Fusion instance not set to ENU.

To use Fusion directly without the estimator, set `settings.convention = FusionConventionEnu` and copy the arrays. Fusion's quaternion is w,x,y,z and its vectors are x,y,z, the same layout as `AtlasCanardInput`. The gyro is in deg/s and accelerations are in g, the units Fusion uses.

**Speed.** With `airspeed_m_s < 0`, speed is estimated by integrating Fusion's earth acceleration. Integration starts once linear acceleration exceeds `launch_accel_g` for `launch_hold_us`; call `AtlasCanard_Reset()` on the pad.
- This estimate is only reasonable over a short flight.
- It undercounts if the accelerometer saturates during boost. The LSM6 tops out at ±16 g, so use the ADXL375 or a baro-derived speed if the boost exceeds that.

## Parameters to fill in

`AtlasCanard_DefaultConfig()` sets **placeholders**. The current Apollyon-I-based estimates, their sources and open questions are in [Canard parameters](CANARD_PARAMETERS.md). Replace them when the airframe is known:

| Field | Meaning |
|---|---|
| `canard_count`, `canard[i].azimuth_deg` | Number of canards (3–6) and their angle around the body from body +X toward +Y |
| `canard_area_m2`, `canard_lift_slope_per_rad` | Area of one canard and its lift slope (CLα per radian; about 2π·AR/(2+AR) for a thin fin) |
| `canard_arm_m` | Distance from the CG forward to the canard centre of pressure (negative if behind the CG). Use the CG at the time the canards are active. |
| `canard_radius_m` | Body axis to the canard centre of pressure |
| `inertia_kg_m2` | Pitch/yaw (X, Y) and roll (Z) moments of inertia about the CG |
| `sensor_from_body` | IMU mounting. Identity means the sensor axes are the body axes, with sensor +Z toward the nose. |
| `canard[i].servo_center_us`, `servo_us_per_deg` | Servo trim, and scale with sign. Set the sign so a positive deflection gives lift toward increasing azimuth. |
| `max_deflection_deg`, `max_rate_dps` | Mechanical limit and servo speed |
| `min_dynamic_pressure_pa` | Below this, the canards stay neutral (default 500 Pa ≈ 29 m/s) |

## Tuning

- `kp_tilt = ωn²`, `kd_tilt = 2·ζ·ωn`. The default is ωn = 6 rad/s and ζ = 0.7 (36 and 8.4).
- Keep ωn well below the servo bandwidth: about 5× lower than 1/(servo time constant).
- Roll defaults to rate damping only (`kd_roll = 10`). For roll-angle hold, set `kp_roll`, e.g. 16 with `kd_roll` 5.6.
- Leave `ki_*` at 0 unless a steady offset appears. The tilt integrator is in body axes, so use it only while roll is held.
- A fin-stabilized rocket already resists tilt aerodynamically. The controller adds to that, so start with the defaults or lower.

[`test_canard.c`](../Tests/host/test_canard.c) flies a simulated rocket with no fin stability and a 20 ms servo lag. Starting from a 10° tilt and 90°/s roll, it settles in 2–3 s, including when the real canard effectiveness is half or double the configured value.

[`test_canard_fusion.c`](../Tests/host/test_canard_fusion.c) runs the whole chain:
- The real estimator and Fusion settle on the pad, then latch launch.
- The rocket is kicked at 30/−20/90 °/s about X/Y/Z.
- From then on attitude comes from the gyro alone.
- The controller returns the **true** attitude to under 0.2° of tilt.

## Roll damp-and-hold test (ServoBench)

This is the first canard test: **roll (Z) only**.
- The controller drives the roll rate to zero. It does not care which angle the rocket ends at.
- Once the rate has stayed below 3 °/s for 200 ms, it latches that angle and holds it (`kp_roll`).
- A small disturbance is pushed back to the held angle.
- A spin above 15 °/s releases the hold. The controller damps again, then latches a new angle.
- No pitch or yaw command is ever issued: all four canards always get the same deflection.
- **Coupled canards:** all four always receive the same deflection (each channel's reverse sign still applies), as the mechanically coupled canards require.
- **Soft start:** every time control engages (SW2 ON, or after any inactive state) command authority ramps linearly from 0 to 100 % over 1 s (`soft_start_us`; 0 disables it, at most `ATLAS_CANARD_MAX_SOFT_START_US`, 10 s). The integrator does not run during the ramp.
- **Slew cap:** each canard moves at most 200 °/s (`max_rate_dps`). Configurations above the hard ceiling `ATLAS_CANARD_MAX_SLEW_DPS` (600 °/s) are rejected.

It runs inside the existing [four-servo stabilization mode](STABILIZATION.md), on PCB PWM 7, 8, 1 and 2. Every protection of that mode stays in force:
- SW2 must be switched ON, after being released.
- The PWM-rail voltage ceiling applies.
- The estimator must be ready.
- Stop, cancel and emergency fences apply.
- All four servos start together at 1500 µs.
- Commands go out every 5 ms.

Any invalid controller input stops PWM and latches a stabilization fault. Only the target source changes. See `io_roll_test_start()` and `io_roll_test_step()` in [`atlas_io.c`](../App/Src/atlas_io.c).

**Build and flash.** It is the ServoBench image with one CMake flag:

```
cmake --preset ServoBench -DATLAS_CANARD_ROLL_TEST=ON
cmake --build --preset ServoBench
```

Flash it with the dashboard (Firmware → ServoBench) as usual. Its hello message includes `"canard_roll":true`; ordinary ServoBench does not. To go back, reconfigure with `-DATLAS_CANARD_ROLL_TEST=OFF` and rebuild. The flag is stored in the build folder, so it stays on until you turn it off. `Build Atlas ServoBench.cmd` always builds the ordinary image.

**Bench procedure.**
1. **Calibrate upright** (USB-C up, still) in the Stabilization tab. "Upright" becomes the canard nose axis, and the STM32 face is azimuth 0. The canards sit at 45/135/225/315° clockwise looking down from USB-C, which is the same layout as the stabilization demo.
2. **Enable,** then switch SW2 ON.
3. **Spin the board about the vertical (USB-C) axis.** During the first second after SW2 ON, the horns respond only gently (soft start). After that, all four horns deflect together against the spin: more deflection the faster it spins, up to 10°. Stop spinning: about 0.2 s later the horns settle and hold.
4. **Nudge** the board a few degrees: the horns push back toward the held angle. **Spin it again:** the hold releases, the horns damp, and a new angle is held.
5. **SW2 OFF** stops PWM.

Notes:
- **Scale:** with no airflow, the bench uses a fixed nominal 60 m/s airspeed. That gives about 2° of deflection at 90 °/s. Hold stiffness is gentle (`kp_roll` 16), about 0.4° of deflection per 10° of error.
- **Direction:** if any horn moves *with* the spin, flip it with the existing Reverse direction setting for that channel. The saved reverse bits set each canard's `servo_us_per_deg` sign.
- **Drift:** the held angle is gyro-integrated (no magnetometer), so it drifts with any leftover gyro bias. That is fine for a test, but recalibrate if the hold wanders.
- **Flight:** this is a bench mode. In flight, the same controller needs the normal-profile PWM path, a flight-phase enable and a real airspeed.

## Before any flight

1. **Sign check on the bench.** Hold the board, tilt the nose about +X and command with a fixed `airspeed_m_s`.
   - The canards on ±X must deflect so that their lift pushes the nose back toward the target.
   - Spin about the nose: the canards must oppose the spin.
   - Fix any wrong canard by flipping the sign of its `servo_us_per_deg`.
2. Connect the output to the servo PWM only behind the existing output gating (pyro and PWM inhibits, a flight-phase `enable`) and the watchdog design. That integration and its safety review are still to do.
