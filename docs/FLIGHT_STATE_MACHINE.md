# Flight state machine

The normal (flight) image runs five phases. Canard control is on **only during upward coast**.

| Phase | Enters when | State estimation (Fusion) | Canards | Radio |
|---|---|---|---|---|
| **PAD** | Power-on | Calibrating: gyro + accelerometer (+ magnetometer if enabled), gyro bias learned while still | Neutral (PWM on at neutral, if qualified) | Telemetry |
| **ENGINE_ON** | ADXL375 \|a\| > 5 g for 100 ms | Gyro only (+ magnetometer if enabled) (accelerometer saturates) | Neutral | Telemetry |
| **COAST** | ADXL375 \|a\| < 1 g for 250 ms | Gyro only (+ magnetometer if enabled) (accelerometer reads drag, not gravity) | **Roll control** | Telemetry |
| **CHUTE** | Barometric altitude decreasing for 10 s (backstop: 60 s after coast began) | Gyro only (+ magnetometer if enabled) | Neutral at once, PWM signal off 20 s later | Telemetry |
| **LANDED** | Barometric climb rate within ±1 m/s for 10 s (backstop: 15 min after CHUTE began) | Gyro only (+ magnetometer if enabled) | Signal off | Telemetry, **buzzer beep every 2 s** |

Phases only move forward. A reset (power cycle or watchdog) starts again in PAD.

The dashboard names them for the team's list: PAD is shown as **CALIBRATING** until calibration is ready, ENGINE_ON as **BOOST** and CHUTE as **DESCENT**. **Apogee** is an event, not a phase: the time of the highest filtered altitude after launch is recorded (`apogee_ms`) and shown with the peak altitude.

## Two separate jobs

- **State transitions** decide the phase. They use only the **ADXL375** (high-g accelerometer, magnitude of all three axes, so its mounting does not matter) and the **MS5611** barometer. Code: [`atlas_flight.c`](../App/Src/atlas_flight.c).
- **State estimation** tells the controller where the rocket points and how fast it spins. It uses only the **LSM6DSV16B** (gyro and accelerometer) and the **MMC5983MA** magnetometer through the [Fusion](https://github.com/xioTechnologies/Fusion) AHRS in ENU. Code: [`atlas_flight_estimator.c`](../App/Src/atlas_flight_estimator.c).

The phase decides which sensors the estimator may use, and whether the canards may act. Neither job uses the other's sensors.

## Transition details

- **Dwell.** A threshold must hold continuously on fresh ADXL samples. One sample on the wrong side, or an ADXL sample older than 100 ms, restarts the count. This rejects handling bumps on the pad and brief dips during boost.
- **Descent.** The barometer pressure becomes altitude above the pad. The pad reference pressure is averaged on the pad and frozen at launch. An alpha-beta filter tracks altitude and climb rate. "Decreasing" means the filtered climb rate is below −1 m/s on fresh samples. Any sample that is not decreasing restarts the 10 s count, so a brief transonic pressure error cannot deploy.
- **Backstop.** If the barometer never confirms descent, CHUTE starts 60 s after coast began. Every fallback moves toward "control off", never toward "control on".
- **Landed.** In CHUTE, "steady" means the filtered climb rate is within ±1 m/s on fresh samples. Any sample outside restarts the 10 s count, so a pause under the parachute (such as main deployment) does not count. Without the barometer, LANDED starts 15 min after CHUTE began. LANDED only starts the buzzer heartbeat.
- **All thresholds** are in `AtlasFlightConfig` (`AtlasFlight_DefaultConfig()`).

## Pre-flight notes on the thresholds

- **Burnout at < 1 g.** Right after burnout, drag deceleration can exceed 1 g at high speed, so COAST starts only once drag has fallen below 1 g. Control begins later than true burnout. Lower `coast_accel_g` only with flight data.
- **ADXL375 offset.** Its zero-g offset can be a few hundred mg per axis. Check the pad reading: it should be close to 1.00 g. A large offset moves both thresholds.
- **Motor failure on the pad.** If ENGINE_ON triggers but the motor does not lift the rocket, the pad reading (about 1 g) can satisfy "< 1 g" and enter COAST. The canards stay neutral because the speed is about zero (below the dynamic-pressure gate), and the 60 s backstop ends the coast.

## Estimator

- **Timing.** Every LSM6DSV16B FIFO sample is fused with its own measured interval from the sensor's microsecond timestamp. Fusion is told the true 480 Hz rate, so its rejection timeouts are in real seconds. Host test: 90 °/s for 1 s integrates to 90.17° at 480 Hz and with uneven sample spacing.
- **Pad calibration.** The gyro bias is averaged while the rocket is still (|a| within 0.05 g of 1 g, low rate), with a 10 s time constant. `Ready` needs at least 2 s of still data and the end of Fusion's 3 s startup. If the estimator was not Ready at launch, the canards stay neutral for the whole flight.
- **Magnetometer: off by default.** The flight estimator uses the same setting as the stabilization estimator: CMake option `ATLAS_STAB_USE_MAGNETOMETER` (default OFF) with `ATLAS_STAB_MAG_TO_IMU` / `ATLAS_STAB_MAG_OFFSET_GAUSS` in [`atlas_stabilization.h`](../App/Inc/atlas_stabilization.h). Those are **unverified placeholders** (identity, zero); the PCB places the MMC5983MA at 0° and the LSM6DSV16B at 180°, so identity is probably not the real map. Off, roll is gyro-only after launch: an unlearned 2 °/s bias drifts the held roll angle 40° in 20 s; roll-rate damping is unaffected. On (and with a correct map) it stays within about 4°.
- **Nose axis.** While the rocket stands still on the pad, the mean "up" seen by the LSM6 is the rocket's long axis. At launch the canard controller's roll axis is set from it (`AtlasCanard_MountFromAxes`), so the board can be mounted in any orientation. If no pad still time was recorded, the canards stay neutral.
- **Speed** for the canard dynamic-pressure estimate is a single integration of the **ADXL375** (high-g, ±200 g, so boost never saturates it) along the nose. No ADXL375-to-IMU axis map is needed: on the pad its own mean reading gives the nose direction in its axes and its axial offset (|mean| − 1 g). In flight, gravity along the nose is cos(tilt) from the Fusion attitude. It is used only by the canard controller and is allowed to drift. On the pad it leaks to zero, so it only carries the first instants of boost. Host test: ADXL axes deliberately different from the IMU's, 0.1–0.2 g offsets on every axis, 25 g boost for 1 s: 233.7 m/s against a true 235.4 m/s.

## Sensor timing check

| Sensor | Device rate | Read by firmware | Used by | Verdict |
|---|---|---|---|---|
| LSM6DSV16B | 480 Hz FIFO, sensor timestamps | FIFO drained every 1 ms I/O cycle into a 128-sample ring (267 ms of margin) | Estimator: every sample, own interval | OK |
| MMC5983MA | 200 Hz bandwidth (4 ms conversion) | Started every 10 ms, collected next cycle (100 Hz) | Estimator: latest sample within 50 ms | OK |
| ADXL375 | 400 Hz | Polled every 2 ms; snapshot latest | FSM and the speed integration at 100 Hz (about every 4th sample); both use the sample time | OK (speed may drift) |
| MS5611 | OSR 1024 | One pressure + temperature pair every 100 ms (10 Hz) | FSM alpha-beta filter tuned for 10 Hz | OK |
| Application | — | 100 Hz | Canard step on the newest fused sample | OK |
| Canard PWM | 333 Hz frame | One multi-channel command per 10 ms; I/O task executes one command per 5 ms | Update latency at most about 10 ms | OK |

Findings fixed in this change:
- The bench estimator ([`atlas_stabilization.c`](../App/Src/atlas_stabilization.c)) tells Fusion 240 Hz and uses millisecond timestamps. That is right for the bench image (240 Hz data-ready mode), but at the flight image's 480 Hz FIFO its Fusion timeouts would run at half time. The flight estimator therefore uses the 480 Hz rate and microsecond timestamps.
- The bench estimator latches launch from the LSM6 at 3 g and **resets itself** on 2–3 g samples before that latch. During a real boost ramp this could leave it without an attitude. The flight estimator never resets in flight; the FSM alone decides the phase.

Open timing issue:
- **Radio TX blocks the I/O task.** RFD900x writes use blocking UART at 57600 baud. Telemetry sends 50-byte chunks (8.7 ms each), 20 per second, during which ADXL polling and the FIFO drain pause (the FIFO buffers this; ADXL skips a few samples). This fits the 20 ms I/O deadline but uses about 17 % of the I/O task's time. A DMA or interrupt-driven UART write would remove it.

## Canard outputs

`FLIGHT_OUTPUTS_QUALIFIED` is 1 for the test flight, so the image drives the canards on PCB PWM 7, 8, 1 and 2 with the limits in [`atlas_flight_app.c`](../App/Src/atlas_flight_app.c): 1400/1500/1600 µs (±10° around the KST neutral). The controller itself stays within ±5° (1450–1550 µs).

- **One roll canard.** Only `FLIGHT_ROLL_CANARD` (index into the channel list; 0 = PWM 7) follows the roll controller. The other three are always held at their `flight_pwm[]` neutral (0°). The controller assumes four canards share the roll moment, so its canard area is set to a quarter of the real one: it then commands the deflection one canard needs, still capped at ±5°. A single deflected canard also pushes the rocket sideways (pitch/yaw); aero should accept this.

- **Neutral means held.** The servo keeps receiving the 1500 µs pulse, so it actively holds the canard straight against air load. The canards are held at neutral on the pad, during boost, whenever control is off, and whenever outputs are blocked (stale sensor, emergency latch, brownout, hardware fault). New commands are refused until outputs are healthy again. The PWM timers keep the neutral pulse running in hardware even through a fatal fault, until the watchdog reset.
- **On the pad,** the channels are enabled at neutral as soon as outputs are permitted (all four flight sensors fresh, 3.3 V rail in range, servo rail 4.8–8.55 V at enable).
- **Each cycle,** one command sets all four channels together (`ATLAS_IO_PWM_SET_MULTI`): the roll canard's pulse and three neutrals.
- **In CHUTE,** the canards return to neutral. 20 s later the PWM is disabled; that is the only time the signal is removed.

The servos are open loop (no position feedback); see [Canard control](CANARD_CONTROL.md#servo-interface-open-loop-no-feedback).

## Telemetry (LoRa connector / RFD900x)

Telemetry runs in every phase, about once per second. It uses the bench's 512-byte snapshot format with erasure repair ([remote telemetry](REMOTE_TELEMETRY.md)), with `TM_PROFILE` = 2 (flight). The sensor, GNSS and PWM words keep their bench meaning; the LSM words hold the estimator's accelerometer and bias-corrected gyro. The BNO085 words are unused, so they carry the flight state:

| Words | Content |
|---|---|
| `TM_BNO_COUNT` +0..3 | Phase, transition reason, ms in phase, launch tick |
| `TM_BNO_TIME` +0..3 | Altitude (cm), climb rate (cm/s), max altitude (cm), ADXL \|a\| (mg) |
| `TM_BNO_ACCURACY` | Bits: estimator started (0), ready (1), mode (2), ready at launch (3), canard state (8–15), signal off (16), roll holding (17), saturated (18), outputs qualified (19), outputs permitted (20) |
| `TM_BNO_ACCEL` +0..2 | Speed along the nose (cm/s), ADXL375 acceleration along the nose (mg), cos(tilt) (×1000) |
| `TM_BNO_GYRO` +0..2 | Body rates used by the controller (mdps) |
| `TM_BNO_MAG` +0..2 | Canard deflection (mdeg), soft-start authority (×1000), dynamic pressure (Pa) |
| `TM_BNO_Q` +0..3 | Fusion quaternion w,x,y,z (×10⁶) |
| `TM_BNO_IO` | **Calibration word**: started (bit 0), Fusion converged (1), gyro bias done (2), **READY (3)**, currently still (4), bias progress 0–100 % (bits 8–15) |
| `TM_BNO_PROTOCOL` | Still time accumulated for the bias (ms) |
| `TM_FAULT` | Liveness fault (bits 0–7, board about to reset if nonzero) and why outputs are blocked right now (bits 8–15, `AtlasRtosFault`, 0 = permitted) |
| `TM_ERRORS` +0..3 | Sensor read errors, board-service errors, slow I/O cycles, late 100 Hz cycles |
| `TM_COUNT` +0..3 | Last and worst duration of the 100 Hz step (µs, budget 10 000), IMU samples fused, magnetometer samples used |
| `TM_STABILIZATION` | IMU gaps |
| `TM_UP` +0..2 | Gyro bias (mdps) |
| `TM_INIT` +0..10 | ADXL375 pad mean (mg, 3 words), nose in IMU axes (×1000, 3 words), LSM6 accel saturations, pad still time (ms), canards mounted at launch (0/1), apogee tick (ms, 0 before launch), roll canard index (0–3) |
| `TM_SD`, `TM_SD_ERRORS`, `TM_SERVICE`, `TM_STARTUP` | SD log: card detected (bit 0), mounted (1), logging (2), file number (bits 16–31); records written; append failures; records dropped |
| `TM_GNSS_TIME`, `TM_GNSS_FIX` … `TM_TOW` | GNSS fix, satellites, latitude/longitude, MSL height, accuracy, time of week (bench meaning). The dashboard shows them in the Flight state panel and Remote Atlas → GNSS |

**Launch readiness.** Calibration takes at least about 3 s of the rocket sitting still on the pad after power-up (Fusion's startup plus 2 s of still data for the gyro bias); progress only accumulates while still. Wait until the ground sees READY (bit 3 of `TM_BNO_IO`). If the rocket launches before READY, the canards stay neutral for the whole flight.

The ground station decodes profile 2 snapshots as flight data: **Remote Atlas → Flight state** shows a READY TO LAUNCH badge (calibrated, phase PAD, outputs permitted) and the values above. The receiving board runs Bringup or ServoBench 1.5.0 or later with the paired RFD900x.

## SD flight log

The flight image logs to the SD card from power-on, in every phase, through the existing storage owner (lowest-priority task):

- **File:** at boot it picks the first unused name `FLT0000.BIN`, `FLT0001.BIN`, … (one file per boot).
- **Records:** one 512-byte snapshot every 100 ms, in exactly the radio telemetry format with its CRC-32 (5 KB/s, about 18 MB per hour). Each record is synced to the card, so a power cut loses at most the record being written.
- **Never blocks flight:** requests are queued without waiting. A missing, full, slow or removed card only drops records; the board retries mounting once per second while a card is detected. Status goes to the ground in every snapshot (**Flight state → SD log**).
- **Reading it:** copy the file from the card and run `.venv\Scripts\python.exe tools\bringup\flight_log.py FLT0001.BIN`. It writes `FLT0001.csv` with one row per record and one column per value, skipping records whose CRC fails.

## Watchdog and output supervision

A reset mid-flight restarts the board in PAD with no attitude, so the canards would stay neutral for the rest of the flight. The watchdog therefore resets the board **only for a software hang**:

| Condition | Effect |
|---|---|
| I/O, 100 Hz or output task stops running; stack nearly exhausted; assert; long radio/BLE maintenance overruns | Watchdog reset after about 2 s |
| Module missing at boot (GNSS, BLE, radio, buzzer) | Reported only. Outputs need only ADXL375, LSM6DSV16B, MMC5983MA and MS5611 to have started |
| Sensor read error | Counted; the stale sample pauses outputs until the sensor recovers |
| GNSS silent, GNSS/radio/BLE UART error | Counted only (GNSS is not required) |
| One I/O cycle over 20 ms | Counted only |
| One late 100 Hz cycle | Counted only; control resumes on the next cycle |
| Emergency stop latched; brownout, RAM ECC error, ADC/DMA hardware fault, hard fault | Canards held at neutral for the rest of the boot (pyro cut); no reset, except a hard fault, which resets via the watchdog |

Radio telemetry keeps running through all of these except a liveness fault. "Outputs paused" means the canards are held at neutral while the condition lasts; control resumes once outputs are permitted again.

## BNO085

The BNO085 is compiled out by default (CMake option `ATLAS_USE_BNO085`, OFF). Its driver, the CEVA SH-2 stack, its report cache and its snapshot fields are not built. It is not probed and not required, so it cannot inhibit outputs or stop the watchdog refresh. This freed about 7 KB of RAM and 6 KB of flash. The bench dashboard still receives an empty `bno` object, so its schema is unchanged.

## Before the pad (remaining items)

There is no prior flight data, so this image goes straight onto the rocket. Do these in order.

**Build and flash**
1. Build the flight image: `cmake --preset Release` then `cmake --build --preset Release`. Program `build/Release/Atlas.hex` over factory USB DFU exactly as in [startup](startup.md#4-program-the-stm32-over-usb-dfu) (BOOT0 high, STM32CubeProgrammer). The flight image has no USB maintenance mode, so the dashboard cannot update it; returning to a bench image also needs BOOT0 DFU.

**If the canard direction was not checked:** flash the held-straight image instead: `cmake --preset Release -B build/ReleaseNeutral -DATLAS_FLIGHT_ROLL_CONTROL=OFF` then `cmake --build build/ReleaseNeutral`, and program `build/ReleaseNeutral/Atlas.hex`. All four canards stay held straight for the whole flight; phases, estimation, SD log, telemetry and the landed buzzer are unchanged. A reversed roll canard would add spin instead of removing it, so never fly active control unchecked.

**Canard direction and trim (with the ServoBench roll test, before flashing the flight image)**

2. Mount the board in the rocket as it will fly. Flash ServoBench with `-DATLAS_CANARD_ROLL_TEST=ON` ([Canard control](CANARD_CONTROL.md#roll-damp-and-hold-test-servobench)), calibrate with the rocket standing nose-up, and spin it by hand about its long axis. Set Reverse on any channel whose canard moves with the spin.
3. Set `FLIGHT_ROLL_CANARD` to the roll canard and copy its Reverse bit into `flight_direction[]` in [`atlas_flight_app.c`](../App/Src/atlas_flight_app.c): Reverse set → `-1.0f`. The flight image does not read the SD settings.
4. Check that 1500 µs puts each canard straight. If not, change that channel's neutral in `flight_pwm[]` (10 µs per degree) and keep minimum/maximum about 100 µs either side. Rebuild Release.

**On the pad**

5. Insert the SD card. Power up and keep the rocket still. In the dashboard (**Remote Atlas → Flight state**), wait for **READY TO LAUNCH**; it needs about 3 s of stillness after power-up. Check the canards are held straight and stiff.
6. Check **ADXL375 pad reading** is near 1000 mg; a large offset shifts the 5 g and 1 g thresholds. Check **SD log** shows a file name and a rising record count (not "NO CARD"). Launch is not blocked without a card, but the flight is then recorded only by radio.
7. Expect the canards to stay neutral on the pad: control needs COAST and enough speed (dynamic pressure above 500 Pa, about 29 m/s).

**Known limits for this test flight (accepted)**
- **Magnetometer off.** Roll is gyro-only after launch, so the held roll angle drifts with any leftover gyro bias; roll-rate damping is unaffected.
- **SD log at 10 Hz.** Fast events (boost onset, burnout) are only resolved to 100 ms in the log; the estimator and controller themselves run on every sample.
- **Airframe numbers are Apollyon I guesses** ([Canard parameters](CANARD_PARAMETERS.md)). They set how strongly the canards respond, within the ±5° limit.
- **Late burnout detection.** COAST starts only once drag falls below 1 g.
- **Servos are open loop.** A stalled or reversed canard is invisible except in the roll rate.
- The parachute is handled by the COTS altimeter; CHUTE only ends canard control.
- **Landing detection is barometric.** Wind gusts on the ground can move the pressure enough to restart the 10 s count; the 15 min backstop still ends in LANDED.

Ground tests for all of the above: [Ground test](GROUND_TEST.md).

## Deferred items

Left out of the first flight on purpose (small value now, or not testable in time). In priority order:

1. **RunCam Split V4 trigger over UART4.** The RunCam Device Protocol start-recording command is unverified on the Split V4, and wiring and logic levels need a bench test. For now set the camera to record at power-on.
2. **Magnetometer.** Verify `ATLAS_STAB_MAG_TO_IMU` (MMC5983MA at 0°, LSM6DSV16B at 180° on the PCB) and the hard-iron offset, then enable `ATLAS_STAB_USE_MAGNETOMETER`.
3. **3D position and velocity** (GPS + barometer + acceleration). Not used by control; can be computed after the flight from the SD log.
