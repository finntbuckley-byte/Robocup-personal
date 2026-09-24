# CLAUDE.md — Group 23 RoboCup Robot (ENMT301, UC 2026)

Context file for Claude Code. Read this first every session. Items marked **UNVERIFIED** are
known-uncertain: check the hardware/schematic or ask before relying on them, and never
"fix" them silently by picking one option.

---

## 1. What this project is

University of Canterbury **ENMT301 Mechatronics System Design** — Group 23
(Finn Buckley, Anita Avia, Finlay Fairweather-Logie). Supervisor: Chris (Pretty).
Parts/technical support: Julian.

An **autonomous** tracked robot that competes head-to-head in 2-minute rounds: it navigates an
arena, collects **metal target weights**, rejects **plastic dummy weights**, and delivers
targets to its **home base** for double points.

**Scope decision:** the team has opted OUT of Sphero objectives (snitches/bludgers).
Do not add Sphero-chasing behaviour. The TCS34725 colour sensor was dropped from the
design on 2026-09-24, so there's no colour-based base recognition.

### Who owns what (respect this when editing)
- **Finn (the user in this repo):** navigation, sensor integration, FSM, homing.
- **Partner:** motor control and crane/collection — `motor.cpp`, `collection.cpp`.
  These are *minimally modified* on purpose. Don't refactor them; if a change is needed,
  keep it small, explain it, and flag it for the partner.
- **Anita:** figures (software architecture diagrams, sensor layout sketches).

---

## 2. Competition rules that affect firmware (2026 brief v1.0)

Full text: `docs/Brief_rules_annotated.pdf`.

### Scoring
```
Score = 1 x (kg of target weights on board)
      + 2 x (kg of target weights in home base at end)
      + 3 x snitches on board − 3 x bludgers on board
      − 0.5 x (number of dummy weights on board)
      − penalties
```
- **Max 3 target weights on board** at end of round. Each extra target = −1 point, and the
  *heaviest* extras are the ones dropped from the count. Dummies don't count toward the 3
  but each one costs −0.5.
- Targets weigh **1.0 / 0.75 / 0.5 kg** and all look identical.
- Delivered = on the ground, touching the coloured base area, not under robot control.
  Delivered weights are safe from theft; on-board weights can be stolen.
- Weights pushed out of base and not returned don't count.

**Firmware implications:** count targets collected; once at 3 (or late in the round),
prioritise returning to base and releasing; release in base doubles value. Never end
holding >3 targets. Rejecting dummies matters (−0.5 each).

### Round/conduct
- **2 minutes**, fully autonomous, no human intervention, no restarts.
- Robot starts on its base facing a direction chosen by the director per round → don't
  assume a fixed start heading; zero the IMU heading at start.
- Robot must **never leave the arena** (disqualification).
- No deliberate damage to the other robot. Two robots are in the arena at once →
  expect a moving obstacle.
- Must have a defined front end.

### Arena
- 2.4 × 4.9 m, 400 mm high walls. **Walls and floor are black** (bad for analog IR).
- Vertical obstacles (walls, pipes) are **red**. Horizontal obstacles (speed bumps, ramps)
  are black like the floor.
- Speed bumps: rectangular, up to 25 mm. Ramps: up to 100 mm high, max 30% gradient.
- All gaps between walls > 0.4 m.
- Two bases, **600 × 600 mm**, in **corners**, coloured **green and blue**, with a
  ~10 mm rim.

### Weights
- Cylinder Ø50 mm × 70 mm high, annular grip groove. >5 targets per round.
- Targets: steel. Dummies: non-conducting plastic; **some dummies have a steel insert in
  the top** (so magnets will pick them up — the funnel sorter must catch them).
- Weights may sit against walls. Knocked-over weights stay on their side.

### Hardware/safety constraints
- Controller must be the **Teensy 4.0**. Must use the supplied power module between
  battery and electronics. ≤100 V DC. Lasers <5 mW. Spinning parts <200 rpm unless guarded.
- **Elastomer ban:** no rubber bands, strings, strips (>20% stretch, aspect ratio >5).
- $50 extra budget, 500 g PLA. Team requirement: robot ≤400 × 400 mm footprint.

---

## 3. Hardware architecture (confirmed)

| Subsystem | Implementation |
|---|---|
| MCU | Teensy 4.0 on a custom PCB |
| Locomotion | Supplied tracked chassis, two DC motors |
| Motor driver | DFR0513 PPM driver on connector **CON65** (the SERIAL1 port: RX1 = D0, TX1 = D1). Motor 1 = **left**, motor 2 = **right**. Control is servo-style pulses: **1.05 ms full reverse, 1.50 ms stop, 1.95 ms full forward**; the driver ignores pulses outside its valid range |
| Collection | Swing-arm crane with a **multi-magnet array** (upgraded from a single magnet to remove the single-point grip failure) |
| Intake/storage | Funnel intake; hinged **rear flap** releases weights |
| Sorting | Inductive proximity sensor at the funnel end classifies metal vs non-metal |
| I2C | **TCA9548** mux (0x70) + **SX1509** GPIO expander at **0x3F**; the SX1509 drives ToF **XSHUT0–7** (plus BIO8–12) |
| Drive encoders | The main drive motors have **built-in magnetic encoders** (direction + distance), read via the encoder IO board (2 digital lines, special 2 mm 6-pin cable). Not currently wired or used |

Available extra parts (red box, see `docs/ENMT301AdditionalParts2026.pdf`): extra DC motor
drive, 1× VL53L0X + 2× VL53L1X, serial ToF, 2× HC-SR04 ultrasound, DFRobot SEN0628 8×8 ToF
(I2C mode), PMW3901 optical-flow "XY" sensor, camera mount, battery holder.

### Sensor suite (connector map confirmed on the board 2026-09-24)

| Sensor | Qty | Connector | Role |
|---|---|---|---|
| DFRobot **SEN0628** 8×8 matrix ToF | 1 | CON64 (RAW I2C1, `Wire1`), 0x33 | Front obstacle avoidance. **Replaces** the old top-front + angled-corner ToFs |
| **VL53L0X** (short range) | 2 | Left CON29 (XSHUT2), right CON28 (XSHUT1) | Bottom pair, **weight detection** + APPROACH steering |
| **Weight-detect ToF** (VL53L1X per `nav_test`, unconfirmed) | 1 | CON27 (XSHUT0) | Across the notch. It's also the **funnel presence sensor**, replacing the ultrasound and the funnel analog IR |
| **VL53L1X** (long range) | 1 | CON30 (XSHUT3) | Rear, reversing clearance |
| **GP2Y0A21** analog IR (white, 100–800 mm) | 2 | Left CON24 (A9Z, pin 23), right CON23 (A8Z, pin 22) | Side-facing, wall-scrape nudge |
| **Inductive proximity** (LJ18A3-8-Z/BY) | 1 | Via the inductive level-shift board to CON70 (A6Z, pin 20) | Funnel end, metal vs non-metal |
| **IMU** (SEN0253 = BNO055, 0x28) | 1 | **Not fitted for now** | Homing, planned |
| IR beacon | — | — | **Fallback only**, if IMU homing proves insufficient |

The **TCS34725 colour sensor has been dropped** from the design, so base arrival can't be
confirmed by colour. The Herkulex gate servo goes through a digital level-shift board to
**CON67 (SERIAL7)**.

> The code may still reference the old ToF layout (top-front, corners) until the SEN0628
> migration is done. Treat those as legacy.

---

## 4. Firmware

PlatformIO project, Teensy 4.0.

```bash
pio run                     # build
pio run -t upload           # flash
pio device monitor -b 115200
```
All serial debug/characterisation tools use **115200 baud**.

### Board and bus quick facts (from `docs/Parts_Summary_2026B.md`)
The full connector-to-pin tables and module notes are in `docs/Parts_Summary_2026B.md`, sections 1–4. Those pin maps were transcribed from board diagrams, so treat any single pin as needing a check on the real board.

- Teensy pins are **3.3 V only**. Anything 5 V goes through a level-shift board.
- The course docs recommend starting the CPU at **150 MHz** and raising it later, because not everything works at 600 MHz.
- **I2C buses.** RAW I2C0 connectors are `Wire` (SDA 18, SCL 19). RAW I2C1 connectors are `Wire1` (SDA 17, SCL 16).
- **I2C addresses:**

  | Address | Device |
  |---|---|
  | 0x28 | BNO055 IMU |
  | 0x29 | ToF default **and** TCS34725 (clash) |
  | 0x30–0x38 | Reassigned ToF sensors |
  | 0x33 | SEN0628 8×8 ToF |
  | 0x3C | OLED |
  | 0x3E | SX1509 for limit switches / AIO |
  | 0x3F | SX1509 for XSHUT / BIO |
  | 0x70 | TCA9548 mux |
  | 0x71 | Add-on ToF expander |

- The **ToF and TCS34725 clash at 0x29.** Keep them apart with the mux, with separate buses, or by reassigning the ToF addresses before the colour sensor is initialised.
- **Analogue ports:**

  | Port | Teensy pin |
  |---|---|
  | A0Z | 14 |
  | A1Z | 15 |
  | A6Z | 20 |
  | A7Z | 21 |
  | A8Z | 22 |
  | A9Z | 23 |
  | A10Z | 24 |
  | A11Z | 25 |
  | A12Z | 26 |
  | A13Z | 27 |

  Pins 24 and 25 are also Wire2's SCL2 and SDA2, so don't use A10Z/A11Z if `Wire2` is ever used.
- **Digital ports.** DIGITAL RAW1 (CON54) is D30–D33. DIGITAL RAW2 (CON55) is D2–D5.
- **Serial ports.** SERIAL1 (CON65) is D0/D1. SERIAL2 (CON66) is D7/D8. SERIAL7 (CON67) is D28/D29.
- **Required libraries:**
  - SparkFun SX1509 2.0.1
  - Pololu VL53L0X 1.3.1 and VL53L1X 1.3.1
  - Adafruit BNO055
  - Adafruit TCS34725
  - DFRobot_MatrixLidar, for the SEN0628
- **Inductive interface board (503)** needs **11 V** from the power module. Its channels A and B are for the large green sensors; channels C and D are for the small black sensors.
- **Electromagnets** are driven through the FET driver board (504). It supports PWM but can't reverse.
- Hitting a servo arm can send back-EMF down the control line and **reset the CPU**. The 514 servo isolator board exists to stop this, so consider it if the crane servo sees impacts.
- The blue **GO** button can be wired to a 3-pin analogue port and used as the program-start input.

### File map
| File | Purpose |
|---|---|
| `config.h` | Pins, I2C addresses, thresholds, calibration constants. Single source of truth for pins |
| `drive.cpp/.h` | Drive abstraction over the motor driver |
| `motor.cpp` | Partner's motor control (minimally modified — leave alone) |
| `collection.cpp` | Partner's crane/collection logic (minimally modified — leave alone) |
| `nav_main.cpp` | `nav` env entry point: sensor updates, crane, sorting, round, telemetry, serial menu |
| `tof.cpp/.h` | The 4-ToF chain: XSHUT sequencing via raw SX1509 register writes, re-addressing from 0x34, non-blocking reads |
| `x8.cpp/.h` | SEN0628 8×8: obstacle band split into left/right halves, bearing, wall check, stale-frame detection |
| `round.cpp/.h` | GO/auto-start, 120 s timer, targets-on-board cap (`roundWantsWeights()`) |
| `ir_sensors.cpp/.h` | Side GP2Y0A21 IR reads + distance conversion |
| `funnel_sensor.cpp/.h` | Funnel presence (from the weight-detect ToF) + inductive sorting |
| `weight_detect.cpp/.h` | Weight candidate: a bottom ToF return with no matching 8×8 obstacle on that side |
| `navigation.cpp/.h` | Top-level navigation FSM, updated for the 8×8 |
| `gate.cpp/.h` | Herkulex rear-flap (gate) servo |
| `nav_test.cpp` | Drive/nav bring-up rig with the current sensor layout (commented out, `navtest` env disabled) |
| `wire_finder.cpp` | `wirefind` env: I2C, mux, XSHUT and port discovery, drives no actuators |
| `servo_test.cpp` | `servotest` env: crane servo bench test |

Hardware checks still waiting on the robot are listed in `BENCH_TODO.md`.

### Build environments — two parallel builds
- **Default env (`teensy40`)** runs `main.cpp`, the partner's collection test. It starts one crane
  cycle 10 s after boot. Its `build_src_filter` excludes the nav-only files.
- **`nav` env** is the full navigation build (`nav_main.cpp`, written 2026-09-25). It uses the new
  4-ToF + SEN0628 layout. It compiles but **hasn't been run on the robot yet**, and several
  `config.h` values are `TODO(verify)` until the bench checks in `BENCH_TODO.md` are done.
  `gate.cpp` is left out until the pin 28 clash is fixed.
- `servotest` and `wirefind` are standalone bench sketches.

### Navigation FSM (`navigation.cpp`) — 8 states
| State | Behaviour |
|---|---|
| `FORWARD` | Default cruise |
| `TURN_L` / `TURN_R` | Obstacle-avoidance turns |
| `ESCAPE` | Unstick/back-off behaviour |
| `APPROACH` | **PD steering** on the imbalance between the low-front VL53L0X pair, closing on a weight |
| `PICKUP` | Hand-off to crane/collection |
| `REPOSITION` | Re-align after a failed/partial pickup |
| `SCAN` | Look for weights |

Stubs already in place: comments tagged **`IMU HOOK`** and **`BEACON HOOK`** mark where
homing integrates. Implement IMU homing there; keep the beacon path as a fallback stub.

### Key design decisions (keep these intact)
- **Sorting is decoupled from the collection FSM.** The funnel sensor classifies whatever
  passes the funnel end independently of navigation/collection state. Don't couple them.
- Inductive proximity is the only viable metal classifier in the parts catalogue.
- Analog IR performs badly on the black arena (low reflectivity) — known risk; that's why
  the funnel moved to ultrasound and front sensing moved to ToF.
- The drive motors already have built-in encoders, but they aren't wired or used yet. They are the cheapest odometry source for homing, and were recommended as a near-free
  odometry baseline for homing. Worth raising if IMU-only homing drifts.

---

## 5. Test data already collected (from the progress report)

### Inductive sensor (LJ18A3-8-Z/BY) max sensing distance
| Target / orientation | Range |
|---|---|
| Steel weight — bottom, top | 7 mm |
| Steel weight — angled | 6 mm |
| Plastic dummy — any | not detected |
| Plastic w/ steel insert — **top** face | **7 mm** |
| Plastic w/ steel insert — side | not detected |
| Plastic w/ steel insert — side, angled | 1 mm |

**Implication:** the sensor must see the *side/body* of the cylinder, not the top face,
or insert-dummies read as metal. Weights must pass within ~6 mm.

### Ultrasound (as read by the test code, ≈cm units)
- 0–1 mm reads **814** (dead-band / wrap). Treat that value as "object touching",
  not "far away" — important now that ultrasound is at the funnel end.
- ~±1 cm to 500 mm; ±5 cm beyond 500 mm; unreliable past ~700 mm.

### Past-team data (research)
Inductive-sorting teams hit 85–100% sorting success. Crane picked up 83% on the first
attempt but is slow (must stop). Data reporting preference: **raw CRGB counts**, not
normalised values, for colour-sensor data.

---

## 6. Open issues — UNVERIFIED, resolve with the user/hardware

1. ~~Inductive sensor pin conflict~~ **RESOLVED 2026-09-24:** the sensor goes through the
   inductive level-shift board to CON70 (A6Z) = **pin 20**. In `config.h`.
2. ~~IR part-number mismatch~~ **RESOLVED 2026-09-24:** the side IR sensors are **white =
   GP2Y0A21** (100–800 mm). `config.h` uses its curve, and `SIDE_NEAR_MM` was raised from 80
   to 150 because readings below about 100 mm are unreliable.
3. ~~Side IR left/right mapping~~ **RESOLVED 2026-09-24:** left = A9Z (CON24, pin 23),
   right = A8Z (CON23, pin 22). In `config.h`.
4. **I2C bus topology:** the "four I2C groups" on the schematic are the board's four
   **I2C In** connectors. Each feeds one sub-assembly and must be jumper-cabled to a RAW
   I2C port:

   | Connector | Feeds |
   |---|---|
   | I2CA (CON9) | AIO expander, 0x3E |
   | I2CB (CON26) | XSHUT/BIO expander, 0x3F |
   | I2CC (CON35) | ToF connector bus |
   | I2CD (CON44) | TCA9548 mux, 0x70 |

   A cable to RAW I2C0 puts that sub-assembly on `Wire`; a cable to RAW I2C1 puts it on
   `Wire1`. The course photos show all four going to I2C0. **Current code assumes one bus.**
   The team doesn't know how ours are cabled, so run the `wirefind` sketch: its report
   prints which bus the XSHUT expander and each ToF answer on.
5. ~~XSHUT placeholders~~ **RESOLVED 2026-09-24:** bottom-left CON29 (XSHUT2),
   bottom-right CON28 (XSHUT1), weight-detect/upright CON27 (XSHUT0), rear CON30
   (XSHUT3). Top-front and corner ToFs no longer exist. Still to confirm: whether the weight-detect
   ToF is an L0X or L1X (`wirefind` reports it).
6. SEN0628 integration is agreed but not yet in code. It is on **CON64 (RAW I2C1 → `Wire1`)**,
   0x33. The ToF chain now re-addresses from 0x34 to stay clear of it.
7. **Pin 28 clash:** the Herkulex gate is on CON67 = SERIAL7 (RX7 = pin 28, TX7 = pin 29),
   and `collection.cpp` also drives the crane servo on pin 28. `Serial7.begin()` takes pin
   28 over as a UART input, so the crane and the gate can't both work as written. Find
   which connector the crane servo's signal wire is really plugged into. The recommended
   fix is to move the Herkulex cable to CON66 (SERIAL2); see `BENCH_TODO.md`.

---

## 7. Roadmap (roughly in priority order)

1. ~~Restore `navigation.cpp` and `tof.cpp`~~: done in the `nav` env (2026-09-25), untested on the robot.
2. Resolve open issues 4 and 7 against the physical board (see `BENCH_TODO.md`).
3. ~~Integrate SEN0628~~: coded in `x8.cpp`. Still needs orientation and obstacle-band tuning on the robot.
4. Implement **IMU homing** at `IMU HOOK`: zero heading at start (start direction varies),
   track heading back toward the start corner, confirm arrival without colour (the
   TCS34725 is dropped), e.g. with corner geometry from the 8×8 plus the base rim, then open
   the rear flap to deliver.
5. Round strategy: the target cap and 120 s timer are done (`round.cpp`). The time-based
   return-to-base trigger is ready but switched off (`USE_HOMING 0`) until homing exists.
   The GO button still needs wiring (`PIN_GO`).
6. Wire up the drive encoders through the encoder IO board to give homing odometry alongside the IMU heading.
7. ~~Gold Sphero colour calibration~~: no longer needed, since the colour sensor was dropped.
8. Collect report data: sorting/collection accuracy, speed, battery, obstacle-avoidance
   success rate.

---

## 8. Reporting context (Design Progress Report)

Code changes often feed the DPR, so keep the firmware's documented status accurate
(including the temporary debugging state above).
- Supervisor preference: **visual over prose** — annotated figures and tables.
- FTA reinforces claims already in the report; doesn't introduce new ones.
- Current DPR has an FTA with five sub-trees (top-level, collection, sorting/storage,
  locomotion, navigation), Figures 1–5, Appendices A–E.
- Still needed from the team: robot photos/CAD renders, sorting + collection accuracy
  data, speed and battery figures, obstacle-avoidance success rate, CDR requirements table,
  **FSM and software block diagrams**, engineering drawings, per-member AI Use Declaration.
- Open request from Anita: a **sensor field-of-view drawing**.

Reference docs in `docs/`. Each `.md` is a text copy of the PDF with the same name; read it first and open the PDF only for figures.

| File | Contents |
|---|---|
| `Brief_rules_annotated.pdf` / `.md` | Competition rules and scoring |
| `Parts_Summary_2026B.pdf` / `.md` | Parts catalogue, CPU board connector and pin maps, I2C addresses, module and hookup notes |
| `ENMT301AdditionalParts2026.pdf` / `.md` | 2026 additional parts (SEN0628, PMW3901, HC-SR04) |
| `ENMT301_Progress_Report.md` | Progress report draft, including the inductive and ultrasound test data |

The exemplar reports (Groups 11, 18, 27, 40) are not in the repo.

---

## 9. How to work with me

- Brief, structured answers. Say what you changed and why.
- Pins and constants live in `config.h` only — no magic numbers elsewhere.
- When something is UNVERIFIED, ask or add a clear `// TODO(verify):` — don't guess.
- For hardware bring-up, prefer small standalone test sketches (like `IR_LeftRight_Test`)
  over changing the main firmware.
- Keep serial output at 115200 and in a format easy to paste into a table.
