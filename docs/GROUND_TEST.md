# Ground test

Every check to pass before the flight image goes on the pad, in order. Each step says which program or firmware image to use, what to do, and what counts as a pass. Write results in the [results sheet](#results-sheet) at the end.

**Safety for every step:** pyro loads disconnected, J5 open, nothing in the canard path when servos are powered.

## 1. Computer-only tests (no board)

Run on the laptop from the repository folder, in PowerShell:

```powershell
$b="$env:LOCALAPPDATA\stm32cube\bundles"
$env:PATH="$b\cmake\4.3.1+st.1\bin;$b\ninja\1.13.2+st.1\bin;$b\gnu-tools-for-stm32\14.3.1+st.2\bin;$env:PATH"
powershell -NoProfile -ExecutionPolicy Bypass -File Tests\host\run_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tests\bringup\run_bringup_tests.ps1
cmake --build --preset Release
cmake --build --preset ServoBench
```

The host tests need a `gcc` on `PATH`. The bring-up tests need the dashboard's `.venv` (run `Start Atlas Dashboard.cmd --check` once).

| Check | Pass |
|---|---|
| 1.1 Host tests | Ends with `Flight FSM tests passed` and `Flight estimator tests passed`; shows `LANDED at …` about 11 s after touchdown |
| 1.2 Bring-up tests | Exit without `FAIL`; includes `PASS remote UI` |
| 1.3 Both firmware builds | No `warning` or `error`; `build/Release/Atlas.hex` and `build/ServoBench/Atlas-ServoBench.hex` have a new time |

## 2. Rocket board on the bench (ServoBench roll-test image)

Firmware: ServoBench built with `-DATLAS_CANARD_ROLL_TEST=ON`, flashed from the dashboard (**Start Atlas Dashboard.cmd → Firmware update**). Board mounted in the rocket as it will fly, USB-C to the laptop, battery on.

| Check | How | Pass |
|---|---|---|
| 2.1 Sensors | Dashboard → **Sensors** | ADXL375, LSM6, MMC5983, MS5611 all CURRENT; pressure near local value |
| 2.2 Calibration | **Stabilization** tab, rocket nose-up and still → **Calibrate** | "Upright saved" |
| 2.3 Tilt ignored | Enable stabilization, SW2 OFF → ON, tilt the rocket | PWM cards stay at 0° (roll-test image ignores tilt) |
| 2.4 Spin direction (servos attached) | Spin the rocket about its long axis by hand | Every canard tilts **against** the spin; fix with Reverse direction → Save directions. Note each channel's Reverse |
| 2.5 Trim | Untick stabilization; **Servo workbench** → each channel → 0° | Each canard straight at 0°, or note the angle where it is |
| 2.6 Roll canard | Agree with aero which canard does roll (PWM 7, 8, 1 or 2) | Written on the sheet |

Then set in [`atlas_flight_app.c`](../App/Src/atlas_flight_app.c): `FLIGHT_ROLL_CANARD` (0 = PWM 7, 1 = PWM 8, 2 = PWM 1, 3 = PWM 2), that channel's `flight_direction[]` (Reverse ticked → `-1.0f`), and any `flight_pwm[]` neutral trim (10 µs per degree). Rebuild Release (step 1.3).

## 3. Radio link (still on the bench image)

Second Atlas board as the ground station on the laptop's USB; both RFD900x radios on J9; rocket board on battery.

| Check | How | Pass |
|---|---|---|
| 3.1 Link | Dashboard → **Communications → Connect & monitor → Test round trip** | "Connected" |
| 3.2 Remote data | Dashboard → **Remote Atlas** | "Receiving · remote Atlas"; loss under 5 % |
| 3.3 Range | Carry the rocket board 300 m or more away, line of sight | Snapshots keep arriving; note the loss % |

## 4. Flight image on the bench

Flash `build/Release/Atlas.hex` over factory DFU: hold BOOT0 high, plug USB, then

```powershell
& "$env:LOCALAPPDATA\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" -c port=USB1 -d build\Release\Atlas.hex -v
```

Release BOOT0 and power-cycle. The flight image has no USB dashboard; watch it through the ground board: **Remote Atlas → Flight state**.

| Check | How | Pass |
|---|---|---|
| 4.1 Version | Remote Atlas source line | `2.2.0 · Flight` |
| 4.2 Calibration | Rocket nose-up, still, after power-on | Phase shows CALIBRATING, then badge **READY TO LAUNCH** within about 5 s |
| 4.3 Moving blocks ready | Pick the rocket up during calibration | Progress stops; "MOVING"; resumes when still |
| 4.4 Pad reading | **ADXL375 pad reading** | 900–1100 mg |
| 4.5 Servos held | Look at and gently push each canard | All four straight and stiff; pulses about 1500 µs on PWM 7 / 8 / 1 / 2; row shows the right roll canard |
| 4.6 GPS | Outdoors, clear sky, wait up to 5 min | Flight state **GPS** row shows a position, 6 or more satellites, under ±10 m |
| 4.6a SD log | Card inserted before power-on; watch **SD log** row | `FLTnnnn.BIN`, records rising about 10 per second, 0 failed |
| 4.6b SD without card | Power-cycle with no card | Row says NO CARD; READY TO LAUNCH still shown; insert card, row starts logging within a few seconds |
| 4.7 Errors | Flight state rows | Supervisor fault `none`; error counters not rising; control step worst under 10000 µs |
| 4.8 Power endurance | Leave powered at the pad state for the expected pad wait plus 30 min | Still READY; battery voltage (multimeter) above the servo rail minimum |

## 5. Phase walk-through (optional, recommended)

Checks BOOST → COAST → DESCENT → LANDED and the buzzer on the real board. It needs a sustained 5 g for 0.1 s, which hand shaking will not produce (on purpose).

1. Put the powered board (battery attached, servos **disconnected**) in a padded box, tied to a 1 m cord. Clear the area.
2. **BOOST:** swing the box in a circle at 1.5 turns per second or faster for a few seconds (about 9 g). Flight state → phase **BOOST**.
3. **COAST:** stop, then drop the box onto a cushion from about 50 cm (0.3 s of free fall). Phase **COAST**.
4. **DESCENT:** leave it still. The barometer sees no descent, so the 60 s backstop gives **DESCENT** (reason "coast timeout 60 s").
5. **LANDED:** about 10 s later, phase **LANDED** (reason "altitude steady 10 s") and the buzzer beeps every 2 s.
6. Apogee row shows a time; power-cycle and confirm the board is back in PAD/CALIBRATING.
7. **Log check:** power off, copy the newest `FLTnnnn.BIN` from the card, run `.venv\Scripts\python.exe tools\bringup\flight_log.py FLTnnnn.BIN`, and open the CSV. The `flight.phase` column shows 0 → 1 → 2 → 3 → 4 with no gaps in `ms` larger than about 200.

The roll canard may move during COAST here, because the swing makes the speed estimate large; that is why servos stay disconnected.

## 6. Full rehearsal

Everything assembled as for flight, rocket on its stand or rail, ground station at its planned distance. Follow the pad steps in [Flight state machine](FLIGHT_STATE_MACHINE.md#before-the-pad-remaining-items): power on, wait for **READY TO LAUNCH**, check canards stiff and straight, GPS fixed, then power off. Time how long the whole procedure takes.

## Results sheet

| Step | Result (pass / fail / value) | Initials |
|---|---|---|
| 1.1 Host tests | | |
| 1.2 Bring-up tests | | |
| 1.3 Builds | | |
| 2.1 Sensors | | |
| 2.2 Calibration | | |
| 2.3 Tilt ignored | | |
| 2.4 Reverse: PWM 7 / 8 / 1 / 2 | | |
| 2.5 Trim: PWM 7 / 8 / 1 / 2 | | |
| 2.6 Roll canard | | |
| 3.1 Link | | |
| 3.2 Remote data, loss % | | |
| 3.3 Range (m), loss % | | |
| 4.1 Version | | |
| 4.2 READY time (s) | | |
| 4.3 Moving blocks ready | | |
| 4.4 Pad reading (mg) | | |
| 4.5 Servos held, roll canard shown | | |
| 4.6 GPS satellites / accuracy | | |
| 4.6a SD log file / records | | |
| 4.6b No-card behaviour | | |
| 4.7 Errors | | |
| 4.8 Endurance (h), battery V before / after | | |
| 5 Phase walk-through | | |
| 5.7 Log converts, phases 0–4 | | |
| 6 Rehearsal time | | |
