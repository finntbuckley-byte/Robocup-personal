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
| Collection | Swing-arm crane with a **single electromagnet** (reverted from the earlier multi-magnet array - team decision, 2026-09-28). Crane servo moved 2026-09-29 to **pin 15** (was CON67/pin 28 - TODO(verify): which connector pin 15 is on); magnet via the FET board, moved 2026-09-28 to **pin 24 (CON72)** so it's PWM-capable, for a reduced holding level (see `BENCH_TODO.md` 2d) |
| Intake/storage | Funnel intake; hinged **rear flap** releases weights |
| Sorting | Inductive proximity sensor **front-on at the V-notch, ~40 mm up**. It reads metal only when steel is within ~7 mm, so it both **gates the pickup** (steel weight seated in the notch → pick up; anything else → REJECT) and classifies metal vs non-metal. Dummies are rejected *before* pickup, not sorted after |
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
| **Weight-detect ToF** (VL53L1X per `nav_test`, unconfirmed) | 1 | CON27 (XSHUT0) | Across the notch. It's also the **funnel presence sensor** (replacing the ultrasound and funnel analog IR) and the **"bottom"** half of the lying-weight reject pair below |
| **VL53L1X** (long range) | 1 | CON30 (XSHUT3) | Rear, reversing clearance |
| **Baseplate-top ToF** (VL53L0X, proposed) | 1 | CON31 (XSHUT4), addr 0x38 | Bench-confirmed blind to a weight lying on its side, unlike the notch ToF above - the "top" half of the lying-weight reject pair. Now fitted and readable (`u`/telemetry), but the FSM trigger (`navigation.cpp` `lyingWeightConfirmed()`) is **disabled 2026-09-28** (`USE_LYING_WEIGHT_REJECT`) - arena testing found it wasn't working. See `BENCH_TODO.md` 2g |
| **GP2Y0A21** analog IR (white, 100–800 mm) | 2 | Left CON24 (A9Z, pin 23), right CON23 (A8Z, pin 22) | Side-facing, wall-scrape nudge |
| **Inductive proximity** (LJ18A3-8-Z/BY) | 1 | Via the inductive level-shift board to CON70 (A6Z, pin 20) | Front-on at the notch, ~40 mm up. **Pickup trigger only** (2026-09-28: no longer gates REJECT - see below). Active LOW (~500 counts metal, ~3485 clear) |
| **IMU** (SEN0253 = BNO055, 0x28) | 1 | **Not fitted for now** | Homing, planned |
| IR beacon | — | — | **Fallback only**, if IMU homing proves insufficient |

The **TCS34725 colour sensor has been dropped** from the design, so base arrival can't be
confirmed by colour. The Herkulex gate servo goes through a digital level-shift board to
**CON66 (SERIAL2)**. The crane servo moved 2026-09-29 to **pin 15** (was CON67/pin 28) -
TODO(verify): which connector pin 15 is on.

> Only `nav_test.cpp` (commented out, a legacy rig) still references the old ToF layout.

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
| `gate.cpp/.h` | Herkulex rear-flap (gate) servo. **Only for releasing weights at the home base**, not for sorting. Its logic waits until IMU homing + distance estimation exist |
| `nav_test.cpp` | Drive/nav bring-up rig with the current sensor layout (commented out, `navtest` env disabled) |
| `wire_finder.cpp` | `wirefind` env: I2C, mux, XSHUT and port discovery, drives no actuators |
| `servo_test.cpp` | `servotest` env: crane bench test running the real collection FSM, with live tuning |
| `smooth_servo.cpp/.h` | Non-blocking eased (cosine) servo moves for the crane, plus the partner's blocking helper for bench use only |
| `odometry.cpp/.h` | Drive-encoder distance (PJRC `Encoder`), slip-corrected against the 8×8 (forward) / rear ToF (reverse), plus stall detection. Distance only: heading is the IMU's job |
| `encoder_test.cpp` | `enctest` env: encoder pin/sign check by hand, and `ENC_COUNTS_PER_M` calibration over a marked 1 m |

Hardware checks still waiting on the robot are listed in `BENCH_TODO.md`.

### Build environments
- **`nav` is the default env** (`default_envs = nav`), so the VS Code build/upload buttons flash the
  full robot. The old `teensy40` env and its `main.cpp` collection test were retired on 2026-09-27;
  `servotest` covers that job. `main.cpp` and `Inductive_sensor.cpp/.h` aren't built by any env.
- **`nav` env** is the full navigation build (`nav_main.cpp`, written 2026-09-25). It uses the new
  4-ToF + SEN0628 layout and includes the gate (Serial2). **Status 2026-09-25:** it has run on
  blocks. All sensors come up `ok`, GO starts the round, the FSM drove FORWARD/TURN from the 8×8,
  and the kill works. The inductive pickup / CREEP / REJECT logic compiles but is **not yet tested**
  (`BENCH_TODO.md` 2c). Floor tests are waiting on a printed part. `DRIVE_SCALE_PCT` is 50 until
  avoidance is proven.
- **`servotest`** runs the real `collection.cpp` cycle with live tuning over serial:
  `pa/ps/da/ds/ra/rs` set angles and speeds, `+ - ++ -- j` jog, `save p|d|r`, and `s` prints
  config lines.
- **`wirefind`** is a standalone bench sketch that discovers what's connected where.
- **`enctest`** is the drive-encoder bring-up: hand-turn check, then `z`/`f`/`s` over a marked 1 m.
- **Serial Monitor:** use the plug icon in the VS Code status bar. Echo and LF are on for every env.
  Only one program can hold COM3 at a time.
- **Tools:** `tools/serial_log.py` (log a run to `docs/testdata/` as .log + .tsv) and
  `tools/crane_endurance.py` (N crane cycles with prompts to check for heat). Use PlatformIO's
  Python: `%USERPROFILE%\.platformio\penv\Scripts\python.exe`. `pio` isn't on PATH, so use
  `...\penv\Scripts\pio.exe`.

### Navigation FSM (`navigation.cpp`) — 10 states
It only runs while the round is RUNNING (`round.cpp`: WAIT → RUN → OVER, GO button start,
stops at 118.5 s). Obstacles come from the 8×8's left/right halves (rows r2–r4). If the 8×8
goes stale, the robot crawls and ignores weights.

| State | Behaviour |
|---|---|
| `FORWARD` | Cruise with a proportional veer away from obstacles and a side-IR nudge. Picks up straight away if the inductive reads metal |
| `TURN_L` / `TURN_R` | Obstacle-avoidance turns |
| `ESCAPE` | Reverse (rear-guarded) and spin, after repeated flip-flopping turns or when blocked on both sides |
| `SCAN` | Timed spin to look for weights after `SCAN_TRIGGER_MS` without a find |
| `APPROACH` | Steer on the bottom VL53L0X pair: **PD** when both see the weight, otherwise a one-sided arc. The pickup is **not** triggered by distance |
| `CREEP` | The bottom pair lost a candidate within 20 cm (they're blind once it's in the notch), so creep straight for up to 1 s |
| `PICKUP` | **Triggered by the inductive sensor reading metal** (debounced 60 ms). Stop, run the crane, wait for `collection_busy()`. Metal gone afterwards = success (`noteCollected`); still there = retry once |
| `REJECT` | CREEP times out with no metal ever seen (dummy/nothing): reverse ~10 cm, pivot away, suppress the spot 4 s. No longer gated on the inductive sensor. A second trigger, `lyingWeightConfirmed()` (notch ToF sees something but the baseplate-top ToF doesn't - checked in FORWARD/APPROACH/CREEP), exists but is **DISABLED 2026-09-28** (`USE_LYING_WEIGHT_REJECT` in `config.h`) - arena testing found it wasn't working. See `BENCH_TODO.md` 2g |

**Team decision 2026-09-28, tried and reverted same day:** knocked-over (lying) weights are back to just being *ignored* (never read as metal, so REJECT only catches them incidentally via the CREEP timeout). A baseplate-top ToF was added to turn this into an active, fast reject (bottom-sees-something + top-sees-nothing, bench-confirmed blind to a weight on its side) - `lyingWeightConfirmed()` still exists in `navigation.cpp`, but arena testing found the behaviour isn't working, so it's switched off pending another attempt (`BENCH_TODO.md` 2g).
| `REPOSITION` | Turn away after a pickup |

Round strategy: stop collecting at **3 targets on board** (`MAX_TARGETS_ON_BOARD`). Planned: the
3rd target is carried on the magnet at rest, which needs PWM holding (`BENCH_TODO.md` 2d).
Comments tagged **`IMU HOOK`** and **`BEACON HOOK`** mark where homing goes; the
time-based return (`USE_HOMING`) is off until it exists.

### Key design decisions (keep these intact)
- **The funnel classifier stays decoupled from the collection FSM.** `funnel_sensor.cpp` watches
  the weight-detect ToF plus the inductive sensor independently, and is now telemetry only.
  Separately, and deliberately, **navigation reads the inductive sensor to gate the pickup**
  (changed 2026-09-25). The sensor sits at the notch, so it can trigger PICKUP the moment a real
  weight is seated.
- **2026-09-28: REJECT no longer reads the inductive sensor at all.** It used to also gate a
  settle-timer-based back-away check, but that was rejecting real upright weights too fast with no
  independent way to confirm orientation. Inductive is PICKUP-only now. A baseplate-top ToF
  (`TOF_TOP`) was added to replace that check (bottom-sees-something + top-sees-nothing means "not
  standing up," checked directly rather than inferred from settle timing), but its trigger is
  **disabled** (`USE_LYING_WEIGHT_REJECT` in `config.h`) as of 2026-09-28 - arena testing found it
  isn't working. Revisit per `BENCH_TODO.md` 2g.
- Inductive proximity is the only viable metal classifier in the parts catalogue.
- **Crane moves are always non-blocking** (`SmoothServo`). Never use the blocking
  `smoothServoWriteSlow()` in the FSM: it stalls sensors, the round timer and the gate.
- **GO arms only after it's been seen released.** A held, stuck or misread button can't start a
  round (this stopped a real runaway on 2026-09-25).
- Analog IR performs badly on the black arena (low reflectivity), which is a known risk. That's why
  front sensing moved to ToF and funnel presence moved to the weight-detect ToF.
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

### Bench data, 2026-09-25 (logs in `docs/testdata/`)
| Test | Result |
|---|---|
| Notch: steel upright | inductive **metal**, weight-detect **57–61 mm**, bottom ToFs **don't see it** |
| Notch: steel lying | inductive **not metal**, weight-detect doesn't see it: lying weights are ignored |
| Notch: plain plastic upright | inductive **not metal**, weight-detect 70–74 mm |
| Notch: empty | weight-detect 77–112 mm (up to 1.5 m looking out) → `FUNNEL_PRESENT_MM` = 66 |
| Steel 12 cm ahead of the notch | only the **bottom-left** ToF sees it (~139 mm); the 8×8 correctly doesn't |
| 8×8 facing open floor | rows r5–r7 see the floor (~640/490/390 mm) → obstacle rows r2–r4 |
| Side IR at ~150 mm | 2100 counts → 147 mm via the GP2Y0A21 curve |
| GO button | idle ~0.6 V, pressed ~3.0 V → active HIGH |
| Crane (pickup 118° @ 45°/s, drop 40° @ 60°/s, rest 70° @ 100°/s) | ~5.0 s full cycle, clean grip and drop with a weight; endurance loop gave no heat |
| Insert-dummy at the notch | **not tested yet** (none available) |
| Notch ToF, lying weight (28/9, replacement sensor) | side-on **110 mm**, end-on **149 mm**, vs upright 59 mm and empty 196–220 mm in the same scene. Detectable, but "empty" = whatever is ahead (77 mm to 1.5 m seen), so it would need an 8×8 cross-check |

### Ultrasound (as read by the test code, ≈cm units) — **no longer on the robot**
- 0–1 mm reads **814** (dead-band / wrap). Treat that value as "object touching",
  not "far away".
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
4. ~~I2C bus topology~~ **RESOLVED (see below):** the "four I2C groups" on the schematic are the board's four
   **I2C In** connectors. Each feeds one sub-assembly and must be jumper-cabled to a RAW
   I2C port:

   | Connector | Feeds |
   |---|---|
   | I2CA (CON9) | AIO expander, 0x3E |
   | I2CB (CON26) | XSHUT/BIO expander, 0x3F |
   | I2CC (CON35) | ToF connector bus |
   | I2CD (CON44) | TCA9548 mux, 0x70 |

   A cable to RAW I2C0 puts that sub-assembly on `Wire`; a cable to RAW I2C1 puts it on
   `Wire1`. **RESOLVED 2026-09-25 (wirefind):** CON26 (XSHUT expander) and CON35 (ToF bus)
   are cabled to RAW I2C0 → `Wire`. The SEN0628 is on I2C1 → `Wire1`. The mux (CON44) isn't
   connected. ToF models confirmed: XSHUT0 = L1X, XSHUT1/2 = L0X, XSHUT3 = L1X.
5. ~~XSHUT placeholders~~ **RESOLVED 2026-09-24:** bottom-left CON29 (XSHUT2),
   bottom-right CON28 (XSHUT1), weight-detect/upright CON27 (XSHUT0), rear CON30
   (XSHUT3). Top-front and corner ToFs no longer exist. All four positions were confirmed by
   hand on 2026-09-25, and the weight-detect ToF is an L1X.
6. ~~SEN0628 integration~~ **DONE 2026-09-25** (`x8.cpp`): **CON64 (RAW I2C1 → `Wire1`)**,
   0x33. Orientation is confirmed and the obstacle rows are r2–r4. It sometimes needs a retry to
   enter 8×8 mode after a re-flash (handled by `X8_MODE_RETRIES`). The ToF chain re-addresses
   from 0x34 to stay clear of it.
7. ~~Pin 28 clash~~ **RESOLVED 2026-09-25:** the crane servo is on CON67 (pin 28) and the
   Herkulex gate is on **CON66 (SERIAL2)**. `gate.cpp` uses `Serial2`.

---

## 7. Roadmap (roughly in priority order)

1. ~~Restore `navigation.cpp` and `tof.cpp`~~: done in the `nav` env (2026-09-25), untested on the robot.
2. ~~Resolve open issues 4 and 7~~: done 2026-09-25. Next: the pickup-logic bench tests, then
   floor tests once the printed part is fitted (`BENCH_TODO.md`).
3. ~~Integrate SEN0628~~: coded in `x8.cpp`. Still needs orientation and obstacle-band tuning on the robot.
4. Implement **IMU homing** at `IMU HOOK`: zero heading at start (start direction varies),
   track heading back toward the start corner, confirm arrival without colour (the
   TCS34725 is dropped), e.g. with corner geometry from the 8×8 plus the base rim, then open
   the rear flap to deliver.
5. Round strategy: the target cap and 120 s timer are done (`round.cpp`). The time-based
   return-to-base trigger is ready but switched off (`USE_HOMING 0`) until homing exists.
   The GO button still needs wiring (`PIN_GO`).
6. **Distance travelled** for homing: either the drive-motor encoders (via the encoder IO
   board) or the magnitude of the PMW3901 optical-flow "XY" sensor (SPI, RAW SPI CON53).
   Homing = IMU heading + this distance estimate. **Decided 2026-09-27: encoders first**
   (no mounting, the PMW3901 needs an 80 mm+ overhang bracket and is unproven on a black floor).
   Slip is caught against the ToFs (`odometry.cpp`). The PMW3901 stays the backup; a storage
   redesign that leaves room for it is being printed in case the encoders prove unreliable.
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
- **FSM and software block diagrams are deferred until after the competition** (team decision 2026-09-25).
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
