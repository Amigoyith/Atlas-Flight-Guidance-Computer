# Canard controller parameters (2026–27 baseline)

Inputs for [`AtlasCanardConfig`](../App/Inc/atlas_canard.h), based on **Apollyon I (2025–26)** as the structural baseline for the 2026–27 rocket. Apollyon I had no canards, so every canard value below is an **engineering guess** for the structures, propulsion and aero teams to confirm or replace. See [Canard control](CANARD_CONTROL.md) for what each field does.

Confidence: **Known** = taken from team documents; **Estimate** = derived from known values; **Guess** = placeholder chosen to be plausible.

## 1. Airframe (Apollyon I)

| Item | Value | Confidence | Source / reasoning |
|---|---|---|---|
| Body outer diameter | 15.69 cm (6 in ID tube) | Known | Apollyon I BOM (6" ID airframe); OD as measured on the same stock in the Gladius III BOM |
| Nose cone | 6 in Von Karman | Known | Apollyon I BOM; length not recorded (Gladius III: 92.5 cm) |
| Section lengths | mid 22.5 in (57 cm), aft 45 in (114 cm), couplers 12 in | Known | Apollyon I BOM; fore airframe length not recorded |
| Motor | AeroTech M1939W-PS, 98 mm mount, ~30 in long | Known | Spec sheet, BOM (motor tube 99.06 mm ID × 30 in), FAR'26 debrief |
| Fins | 4, 3/16 in thick, tabless | Known | BOM, CAD (`APLYN1_TablessFin`); planform not recorded (Gladius III: root 426 mm, tip 49 mm, semi-span 131 mm) |
| Launch | Launch **tower**, ~0.25 in rail clearance around the body | Known | Spec sheet, FAR'26 debrief |
| Recovery | Main at 11,000 ft or −120 ft/s | Known | FAR'26 debrief (apogee is well above 11,000 ft) |
| Total length | ≈ 3.3 m | Estimate | Section sum, with Gladius III nose and fore lengths standing in |
| Mass at burnout | ≈ 15 kg (range 12–20) | Guess | Tubes ≈ 4.4 kg (Gladius III BOM), plus recovery, avionics, payload/drone and the empty M casing |
| CG at burnout | ≈ 1.95 m from nose tip | Guess | ≈ 58 % of length |
| `inertia_kg_m2` X, Y (pitch/yaw) | **14 kg·m²** | Estimate | m·L²/12 at burnout |
| `inertia_kg_m2` Z (roll) | **0.08 kg·m²** | Estimate | Thin-shell m·r², reduced for the motor casing near the axis |
| Max speed / Mach in coast | **Unknown** | — | Must come from an Apollyon-class OpenRocket run. An M motor in a 6 in airframe may be transonic. |
| `air_density_kg_m3` | 1.0 | Guess | Mean over a ~2–3 km coast |

## 2. Canards (all guesses)

| Field | Value | Reasoning |
|---|---|---|
| `canard_count`, azimuths | 4, **in line with the fins** (0/90/180/270°) | Clears the launch-tower rails the same way the fins do. Offsetting by 45° would keep the canard wake off the fins but would hit the rails. Aero and structures should decide. |
| Planform | root 7 cm, tip 4 cm, span 5 cm, ~6 mm thick | Scaled up from the team's OpenRocket canard study (5 × 5 × 3 cm on the smaller rocket) |
| `canard_area_m2` | **0.00275** (27.5 cm² each) | Area of the planform above |
| `canard_lift_slope_per_rad` | **2.0** | Low aspect ratio (≈ 0.9) with body carry-over, subsonic. Replace with CFD or OpenRocket values, and expect changes near Mach 1. |
| Axial position | ≈ 1.0 m from nose tip (fore airframe) | Behind the nose shoulder, ahead of the payload bay |
| `canard_arm_m` | **0.95** | Canard centre of pressure is about 0.95 m ahead of the burnout CG |
| `canard_radius_m` | **0.10** | Body radius 7.85 cm plus about 0.42 × span |
| `max_deflection_deg` | **10** mechanical, 5 for the first active flight | Physical stops must enforce this, not only software |
| `max_rate_dps` | 300 | Below KST X10 slew speed |
| Hinge line | ≈ 25 % of root chord | Close to the centre of pressure, to keep hinge moment low |
| Servo | KST X10 V8.0, 10 µs/deg, 1500 µs neutral | Bench servo. Check its torque against the hinge moment (2× margin, per the Structures Technical Goal). |

Rough hinge load at 250 m/s and 10° deflection: normal force ≈ q·S·CLα·δ ≈ 30 N per canard. If the centre of pressure ends up 5–15 mm from the hinge, that is 0.15–0.45 N·m (1.5–4.6 kg·cm) before margin.

## 3. What the controller can do with these numbers

From a simulation with no fin restoring moment (the worst case). The controller corrects a 5° tilt plus a 90 °/s roll, with a 30 ms servo lag:

| Speed | q | Max pitch accel. (27.5 cm² canards) | Saturated? | With 75 cm² canards |
|---|---|---|---|---|
| 60 m/s | 1.8 kPa | 0.2 rad/s² | yes, 1.7 s | 0.6 rad/s², 0.4 s |
| 100 m/s | 5 kPa | 0.7 rad/s² | yes, 0.4 s | 1.8 rad/s², 0.1 s |
| 150 m/s | 11 kPa | 1.5 rad/s² | brief | 4.0 rad/s², no |
| 250 m/s | 31 kPa | 4.1 rad/s² | no (6° peak) | 11 rad/s², no (2° peak) |

- **Roll control is easy:** roll is damped from 90 to 9 °/s in 0.2–1.2 s at every speed, even with small canards.
- **Pitch/yaw authority is marginal** for a 15 kg, 3.3 m rocket with 27.5 cm² canards below ~150 m/s.
- **Choices:** larger canards, gentler tilt gains (ωn = 3 rad/s avoids most saturation), or roll-only control at low speed. All three are compatible with the code.

## 4. Open questions for the team

1. **Airframe size.** The Structures Technical Goal (Sept 27) assumes a **3 in airframe**, and the 2026–27 OpenRocket sheet trades **J/K motors**. Apollyon I is 6 in with an M motor. Mass, inertia, arm and speed change by several times between the two, so confirm which one is the baseline.
2. **Canards vs. fin flaps.** The Oct 25 Fluent CFD manual builds a database for a **fixed fin with a movable flap**. The controller supports both: an aft surface just uses a negative `canard_arm_m`. Which architecture is the plan?
3. **Tower clearance:** does any forward surface conflict with the launch-tower rails?
4. **Speed and Mach profile in coast,** from an OpenRocket run of the chosen motor and airframe.
5. **Atlas board location and orientation** (sets `sensor_from_body` and how the IMU sees vibration).
6. **Measured** mass, CG and inertia at burnout, once built.

## Plugging in the CFD linear model

Structures will provide a linear force model once the CFD model is complete. The controller already uses a linear model, so it maps onto existing fields without code changes:

| CFD result | Controller field |
|---|---|
| Canard normal force per radian of deflection, per unit dynamic pressure: F = (dF/dδ/q) · q · δ | `canard_area_m2 × canard_lift_slope_per_rad` = dF/dδ/q (m²/rad). Set the area to the reference area and the lift slope to the CFD slope, or set the area to 1 and the slope to the product. |
| Centre-of-pressure location of the canard force | `canard_arm_m` (distance ahead of the CG) and `canard_radius_m` (from the body axis) |
| Centre of mass (CG) at the controlled phase | Moves `canard_arm_m`: arm = canard CP station − CG station |
| Mass properties | `inertia_kg_m2` |
| Gravity | Not a control input: roll control is unaffected, and the attitude estimate already removes gravity (Fusion) |

A roll-moment coefficient (for example C_l,δ) from CFD replaces the area, slope and radius term: roll moment per radian per pascal = `canard_count × canard_area_m2 × canard_lift_slope_per_rad × canard_radius_m`. Choose the values so that product matches. The force per radian may vary with Mach or angle of attack. If so, give the coefficients at the controlled-phase Mach; the controller already scales with q, and a Mach-dependent table would be a small addition.

## Configuration snippet

```c
AtlasCanardConfig c;
AtlasCanard_DefaultConfig(&c);           /* 4 canards at 0/90/180/270, KST X10 mapping */
c.canard_area_m2 = 0.00275f;             /* GUESS: 27.5 cm^2 each */
c.canard_lift_slope_per_rad = 2.0f;      /* GUESS: replace with CFD */
c.canard_arm_m = 0.95f;                  /* GUESS: canard CP to burnout CG */
c.canard_radius_m = 0.10f;               /* GUESS */
c.inertia_kg_m2[0] = c.inertia_kg_m2[1] = 14.0f;  /* ESTIMATE: Apollyon I at burnout */
c.inertia_kg_m2[2] = 0.08f;              /* ESTIMATE */
c.air_density_kg_m3 = 1.0f;
c.max_deflection_deg = 5.0f;             /* first active flight */
c.kp_tilt = 9.0f; c.kd_tilt = 4.2f;      /* wn = 3 rad/s: avoids low-speed saturation */
```
