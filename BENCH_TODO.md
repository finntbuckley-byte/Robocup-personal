# Robot TODO

Work top to bottom. 🤖 = Claude can do it from the laptop once the Teensy is plugged in (with a
**data** USB cable).

## 0. Next session (from 27–28/9 late)
- [ ] **Notch ToF (CON27, XSHUT0, L1X) resets itself - electrical fault.** Sets up fine, then
      within ~0.5 s of starting to range it drops off the bus and ~2 s later reboots at the default
      0x29, never giving a reading (`UP` always 0). Classic brownout. Worked 25/9 (57–61 mm), so
      something physical changed. Not a code problem: the rear L1X uses the same code and works.
      Diagnostics in the nav build: `u` (per-ToF raw mm / status / data age / I2C error),
      `i` (I2C scan + ToF probe), `r` (re-init the notch ToF and probe it).
  - [ ] Power off, reseat the CON27 cable at both ends; check it isn't pinched near the notch,
        funnel or the new IMU mount. Power up, `u`.
  - [ ] Still failing: swap the notch and rear L1X cables (CON27 <-> CON30). Fault moves to
        index 3 = the sensor is dead (2 spare VL53L1X in the red box). Stays on index 2 = the
        CON27 connector / cable / board supply.
  - [ ] Optional: 3.3 V at the notch sensor's pins.
  - [ ] Once fixed: remove the TEMP diagnostics (the `probeTofs()` boot prints in nav_main.cpp
        and `tofReinitL1X()` / `r`); keep `u` and `i`.
  - Only affects the funnel telemetry - pickup uses the inductive, weight-finding uses the
    bottom ToFs + 8x8 - so it does NOT block avoidance tests.
- [ ] **After the flange reprint:** ruler runs again (enctest GO, hold on). Read the steady
      `integral` from the traces (`T`) → `HEADING_I_START` in config.h (runs that started
      pre-loaded at ~-8 to -10 % were the straightest).
- [ ] Recalibrate `ENC_COUNTS_PER_M` **with the heading hold on**, on the **arena floor**: hold-on
      runs gave ~15,750–15,900 counts/m vs 15,640 without (steering scrub adds counts).
- [x] (record) Heading-hold findings 28/9: IMU matches the ruler within ~1 deg; hold cut the turn from
      8–12 deg to 1–5 deg over ~60 cm. The robot runs straight for ~1 s then starts turning
      right in most runs - partly the floor at the first test spot, partly something on the robot
      (check again after the reprint).
- [ ] 8x8 check on the floor before the obstacle test: `g` - r5–r7 floor ~0.4–0.7 m, r2–r4 clear
      (0 or >1.5 m) with nothing ahead; a box 50 cm ahead shows in r2–r4.
- [x] **First wall test (28/9, log `docs/testdata/2026-09-28_nav_wall_test.log`):** drove at a
      low box from ~68 cm; 8x8 closure matched the encoders (418 vs 435 mm); at ~25 cm it turned
      LEFT towards the open side (correct direction - motor fix confirmed); GO-stop worked.
- [ ] **IMU heading frozen in nav:** `hdg` read 0.0 for the whole wall test despite turning both
      ways (worked in WAIT on the first nav boot; one boot said "IMU not found"). The IMU is
      flaky on `Wire` - check together with the notch ToF fault (same bus). Heading hold was
      effectively OFF in that run.
  - [ ] **Hypothesis: 3.3 V brownout, not the I2C bus.** One cause fits both faults: the notch ToF
        resets the moment it starts ranging (current pulses), and a BNO055 that browns out
        reboots into CONFIG mode where its heading output stops updating (= constant 0.0 while
        the code still thinks it's fine). Heading worked in WAIT (motors off), froze during the
        run (motors on). Both faults appeared after the IMU was fitted (extra load / disturbed
        cables). Nothing run in software could have caused it.
  - [ ] Test: on blocks, motors on (enctest `f`), watch the BNO055 operating mode (reg 0x3D,
        IMUPLUS = 0x08) + the notch ToF; check sensor power/ground cables and the 3.3 V at the
        sensors under motor load.
  - [ ] Software safety net regardless: detect a BNO055 reset (mode register != IMUPLUS) and
        re-init it keeping the heading continuous; same idea for a ToF that falls back to 0x29.
        A sensor rebooting mid-round must not blind the robot.
- [ ] **Low box: hit it, then read it as a weight.** Closing to 258 mm (turn is 250), the box
      dropped BELOW the band's view (reading jumped back to 353/401 mm), so it carried on and
      touched before turning. Then: after the turn it went to APPROACH / TURN_R / ESCAPE chasing
      the box - the bottom ToFs saw it but the 8x8 band looked over it (it's low), which is
      exactly the weight signature. Arena walls are 400 mm so they shouldn't do this, but low
      obstacles / the other robot might. Repeat the test with a wall or tall box (>= 300 mm).
- [ ] Head-on walls: the CAUTION veer barely acts (both halves equal), so it goes straight to
      the 25 cm turn. Fine for now; watch it at higher DRIVE_SCALE_PCT (stopping distance).

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
- [x] Check the tracks turn the right way: motor 1 = left, motor 2 = right. (27/9: were swapped and the
      left was reversed - fixed at the driver/encoder board; wall test turned the right way.)

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
- [ ] **BEFORE COMPETITION: `GO_STOPS_ROUND = false`** (`config.h`). It lets a 2nd GO press stop a
      round for untethered testing (added 28/9), but no human intervention is allowed in a real
      round and a stray press must never end it.
- [ ] Heading hold in nav FORWARD (`USE_HEADING_HOLD`, `HEADING_KP/KD`): tune on the floor. Hold
      proven first in enctest (`H` toggles it), then in nav with the `hdg` telemetry column.
- [x] (done, wired) Wire the **GO button** (e.g. A0Z, CON68) and set `PIN_GO`.
- [ ] Write homing + delivery (IMU fitted 27/9, heading + encoder distance ready). Then turn on
      `USE_HOMING`.
- [ ] Gate/flap sorting logic: keep metal, drop dummies.
- [ ] **Drive encoders (27/9)**: `odometry.cpp` + `enctest` env.
  - [x] Wired: Encoder IO board on **CON55 (D2–D5)**, L A/B = 2/3, R A/B = 4/5 (one board, both motors).
        Found + fixed on the way: motor channel 1 was driving the RIGHT track (leads swapped at the
        driver), and the left motor then ran backwards (polarity flipped at its screw terminal).
  - [x] Signs (enctest, on blocks): `ENC_L_SIGN = -1`, `ENC_R_SIGN = 1` - forward counts up on both.
  - [x] `ENC_COUNTS_PER_M = 15430`: 5 GO-button runs of ~63 cm at cruise, sd 0.4% (not the arena floor).
  - [ ] Recheck counts/m on the **arena floor** (1–2 runs) when it's free.
  - [x] (superseded - the trim isn't the lever: encoders showed equal track speed while it still
        turned, so it's track/floor drag; the heading hold handles it) **Forward trim at cruise:** those runs turned **right ~9°** (6.8–12.2°) and drifted ~5 cm right
        over 63 cm, with `DRIVE_TRIM_L_FWD` 0.90. Part of that was the jolt from the instant stop; the
        soft stop (`DRIVE_DECEL_MS`) is now in. Rerun 3× with enctest (GO) and lower the left trim
        if it still turns right. The IMU heading hold will take out what's left.
  - [ ] Slip check (`nav`, telemetry `odo`/`odoRaw`/`slip`): drive into a wall and hold →
        `odoRaw` keeps climbing, `odo` stops, `slip` = 1. Tune `ODOM_SLIP_*` if not.
- [x] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/` (27/9; the 151 MB PDF is git-ignored, the `.md` is committed).
