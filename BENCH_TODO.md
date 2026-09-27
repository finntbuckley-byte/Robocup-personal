# Robot TODO

Work top to bottom. 🤖 = Claude can do it from the laptop once the Teensy is plugged in (with a
**data** USB cable).

## 1. Bench: robot on blocks, tracks off the ground
- [x] 🤖 **Wire finder** (done 25/9: ToFs + expander on Wire, 8×8 on Wire1, CON27 = L1X): `pio run -e wirefind -t upload`. It finds:
  - which I2C bus the ToFs and the XSHUT expander use. That goes into `TOF_WIRE` / `SX_WIRE` in `config.h`.
  - whether the weight-detect ToF (CON27) is an L0X or an L1X.
  - whether the 8×8 answers on `Wire1`.
- [x] (done 25/9, all match; side IR L/R confirmed too) Cover each ToF in turn (`t`) to confirm: CON29 = bottom-left, CON28 = bottom-right,
      CON27 = weight-detect, CON30 = rear.
- [x] Hold steel at the inductive sensor (`p`) and check pin 20 reads **low** for metal. (done 25/9: ~500 metal, ~3485 clear)
- [x] **Pin 28 clash** (done 25/9: crane CON67, gate CON66 → Serial2): find where the crane servo is plugged in. Recommended fix: move the Herkulex
      cable from CON67 to **CON66**, then 🤖 set `gate.cpp` to `Serial2`.
- [x] (done 25/9, all ok) 🤖 **First nav run**: `pio run -e nav -t upload`. Check all 4 ToFs and the 8×8 report `ok`.
- [x] (done 25/9: rows r2–r4) 🤖 Press `g` to print the 8×8 grid, then set its orientation (`X8_COL_SIGN`, `X8_ROW_FLIP`) and
      obstacle rows (`X8_BAND_LO/HI`), which must see walls but not floor weights.
- [x] (done 25/9: 66 mm) 🤖 Read the empty-funnel `UP` value in the telemetry and set `FUNNEL_PRESENT_MM` below it.
- [x] Speed-ramp test: check the 68% fwd / 73% rev pulse caps. (25/9: team confirmed max 1950 fwd / 1050 rev µs, caps match) Raise them in `config.h` if the tracks
      clearly run faster above that.
- [ ] Check the tracks turn the right way: motor 1 = left, motor 2 = right.

## 2. Crane (with the partner)
- [x] Smooth eased moves integrated (`smooth_servo.cpp`), crane angles/speeds in `config.h`
      (done 25/9: pickup 118° @ 45°/s, drop 40° @ 60°/s, rest 70° @ 100°/s, full cycle ≈5 s).
- [x] Weightless position check + full cycle with a real weight: grips and drops cleanly.
- [x] Watch for servo heat / buzzing after repeated pickups (done 25/9: endurance loop on a plastic dummy, no heat).
- [ ] Add the 514 servo isolator board if the arm takes knocks (a knock can reset the CPU).

## 2b. Dummy rejection (inductive sensor at the notch, 40 mm up, front-on)
- [ ] **Needs an insert-dummy (plastic with a steel top). None found yet.** Upright in the notch
      it should read non-metal (the sensor sees the side, not the steel top). Also try it lying with
      its top facing the sensor, which may false-trigger.
- [ ] Then: pickup only if the inductive sensor reads metal; otherwise reverse about 10 cm and
      pivot away (option C). Only design the flinger if dummies keep getting re-found.

## 2c. New pickup logic (inductive trigger, CREEP / REJECT): bench tests on blocks
Flash `nav`, robot on blocks, say go before pressing GO. 🤖 logs each one.
- [ ] **Pickup chain:** drop a steel weight into the notch mid-round → PICKUP → crane → "OK" →
      `onb` +1 → turns away. Repeat to 3 on board → it stops collecting.
- [ ] **Missed grab:** hold the weight down so the magnets can't lift it → retries once, then gives up.
- [ ] **CREEP → REJECT:** hold a weight ~14 cm ahead of the bottom-left ToF, then pull it away →
      creep, no metal, reverse + pivot. Hand behind the robot during the reverse → rear guard stops it.
- [ ] **Full 2-min round, hands off:** stops itself at ~118.5 s, no hang, no reset (`ms` never jumps
      back to 0). With a pickup in it, this also checks crane + both tracks together don't brown out.

## 2d. Third weight carried on the magnet (held at rest, not dropped)
- [x] Magnet coils **do get hot** in extended use (partner, from earlier testing), so a full-power
      hold for ~100 s isn't safe. Plan: full power to grab, then a **reduced PWM holding level**.
- [ ] **Move the magnets to PWM pins.** Pins 26/27 (CON74/75) have **no PWM** on the Teensy 4.0
      (confirmed from the core's pwm.c). Move to **CON72 (A10Z, pin 24)** and **CON73 (A11Z, pin 25)**:
      both free, both PWM on the same timer. (They double as Wire2, which isn't used.)
- [ ] 🤖 Code: magnet pins → 24/25, `analogWrite` holding level, and a "pick up and hold" crane
      cycle for the 3rd target (grab at full power → rest → drop to holding level). Needs the
      partner's OK (collection.cpp).
- [ ] **PWM hold test:** 1 kg weight held at rest, stepping the holding level down (e.g. 100 → 70 →
      50 → 35 %) to find the lowest that still holds through a shake. Then 2 min at that level:
      feel the coils **and** the crane servo (it holds 1 kg on the arm the whole time).

## 3. On the floor  (waiting on a printed part to improve motion)
- [ ] Obstacle avoidance on its own. `x` kills the motors.
- [ ] **Raise `DRIVE_SCALE_PCT` back to 100** (`config.h`, currently 50 = half the usable pulse
      range, a safety limiter for first floor tests). Once avoidance behaves, step 50 → 75 → 100 and
      retest avoidance at each step: stopping distance grows with speed, so the 8×8 thresholds may
      need lengthening. Time 1 m at 50 and at 100 for the report's speed figure.
- [ ] **Does the V-notch swing actually clear a rejected object?** (REJECT = reverse ~10 cm, pivot,
      ignore the spot 4 s.) The team isn't convinced it works, so test it with a plastic dummy and a
      knocked-over weight. Tune `REJECT_REVERSE_MS` / pivot time / `REJECT_SUPPRESS_MS`. If objects
      don't get pushed clear, or keep getting re-found: longer reverse + bigger turn first, then
      the under-plate flinger.
- [ ] Creep through the blind gap: tune `CREEP_SPEED_PCT` / `CREEP_MAX_MS` so a real weight reliably
      reaches the inductive sensor before REJECT fires.
- [ ] Approach + pickup on a single weight.
- [ ] Tune `SIDE_NEAR_MM` (side IR, currently 150) against a red wall.
- [ ] Paste the telemetry into a spreadsheet for report data: sorting/collection accuracy, speed,
      avoidance success rate.

## 4. Later
- [x] (done, wired) Wire the **GO button** (e.g. A0Z, CON68) and set `PIN_GO`.
- [ ] Fit the IMU and write homing + delivery. Then turn on `USE_HOMING`.
- [ ] Gate/flap sorting logic: keep metal, drop dummies.
- [ ] **Drive encoders (27/9)**: `odometry.cpp` + `enctest` env.
  - [x] Wired: Encoder IO board on **CON55 (D2–D5)**, L A/B = 2/3, R A/B = 4/5 (one board, both motors).
        Found + fixed on the way: motor channel 1 was driving the RIGHT track (leads swapped at the
        driver), and the left motor then ran backwards (polarity flipped at its screw terminal).
  - [x] Signs (enctest, on blocks): `ENC_L_SIGN = -1`, `ENC_R_SIGN = 1` - forward counts up on both.
  - [x] `ENC_COUNTS_PER_M = 15430`: 5 GO-button runs of ~63 cm at cruise, sd 0.4% (not the arena floor).
  - [ ] Recheck counts/m on the **arena floor** (1–2 runs) when it's free.
  - [ ] **Forward trim at cruise:** those runs turned **right ~9°** (6.8–12.2°) and drifted ~5 cm right
        over 63 cm, with `DRIVE_TRIM_L_FWD` 0.90. Part of that was the jolt from the instant stop; the
        soft stop (`DRIVE_DECEL_MS`) is now in. Rerun 3× with enctest (GO) and lower the left trim
        if it still turns right. The IMU heading hold will take out what's left.
  - [ ] Slip check (`nav`, telemetry `odo`/`odoRaw`/`slip`): drive into a wall and hold →
        `odoRaw` keeps climbing, `odo` stops, `slip` = 1. Tune `ODOM_SLIP_*` if not.
- [x] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/` (27/9; the 151 MB PDF is git-ignored, the `.md` is committed).
