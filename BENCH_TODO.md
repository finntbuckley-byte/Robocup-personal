# Robot TODO

Work top to bottom. 🤖 = Claude can do it from the laptop once the Teensy is plugged in (with a
**data** USB cable).

## PRIORITY 1 — integrate with working navigation software
- [ ] PID
- [ ] Collection
- [ ] Homing
- [ ] Gate function
- [ ] stack_test (do last)

## 0. Next session (from 27–28/9 late, plus 29/9 plan)
- [ ] **Lying-weight reject:** now wired back into `nav` (`USE_LYING_WEIGHT_REJECT` in `config.h`)
      with logic ported from `stack_test.cpp` - see §2e for the integration/tuning task.
- [x] (done 29/9) **Integrate a servo board that protects against short circuits** on the crane
      servo line.
- [x] **Notch ToF fault (28/9): the sensor (or its cable) was faulty, not the port.** Swapping
      CON27 <-> CON30 moved the fault with the sensor; a replacement on CON27 then ranged cleanly
      (all 4 ToFs stayed at 0x34-0x37, no errors). Positions re-confirmed: weight in the notch =
      59 mm (same as 25/9, so `FUNNEL_PRESENT_MM` 66 still holds), hand behind = rear 68 mm.
      **Label the faulty sensor and keep it off the robot.** TEMP diagnostics removed; `u`/`i` kept.
- [x] **Flange reprint: done.** Ruler runs / `HEADING_I_START` and `ENC_COUNTS_PER_M` are covered
      by the new §2f (motion/PID calibration) and §4 encoder recheck items - no longer blocked.
- [x] (done 29/9) Recalibrate `ENC_COUNTS_PER_M` **with the heading hold on**, on the **arena
      floor**: hold-on runs gave ~15,750–15,900 counts/m vs 15,640 without (steering scrub adds
      counts).
- [x] (record) Heading-hold findings 28/9: IMU matches the ruler within ~1 deg; hold cut the turn from
      8–12 deg to 1–5 deg over ~60 cm. The robot runs straight for ~1 s then starts turning
      right in most runs - partly the floor at the first test spot, partly something on the robot
      (check again after the reprint).
- [x] 8x8 floor check: done, good.
- [x] **First wall test (28/9, log `docs/testdata/2026-09-28_nav_wall_test.log`):** drove at a
      low box from ~68 cm; 8x8 closure matched the encoders (418 vs 435 mm); at ~25 cm it turned
      LEFT towards the open side (correct direction - motor fix confirmed); GO-stop worked.
- [x] (FIXED 28/9 with a new wire: wiggle test 113/113 OK, both ends flexed) **Notch ToF (CON27) bad connection at the SENSOR END (28/9 wiggle test):** 36/36 OK hands-off,
      dropped the moment the sensor end was flexed; also failed on its own after test 1 (vibration).
      New wire being fitted -> re-run the wiggle test (both ends). If a joint on the module header is
      cracked, reflow it. Add strain relief near the notch.
- [x] **ToF safety net (28/9):** a ToF with no data for 500 ms reads 0 (no frozen old value) and
      `tofOk()` goes false; it's XSHUT-reset + re-initialised (2 s retries, 10 s after 3 failures).
      Other stale sensors are held off 0x29 during a recovery. Tested with the bad cable: healthy
      sensors unaffected, the dropped one came back in 0.3 s. `tofRec` telemetry column.
- [x] **IMU heading frozen in nav (28/9): findings.**
  - The `i` I2C scan itself rebooted the BNO055 (3/3): its empty address-only write to 0x28.
    Fixed: the scan skips reserved addresses and checks the BNO055 by chip-ID read.
  - With the faulty notch ToF unplugged: 0 IMU resets at idle **and** over 20 s of driving on
    blocks, so motor load isn't browning it out. Most likely cause of the wall-test freeze =
    the faulty notch ToF rebooting over and over and dragging the shared 3.3 V.
  - Safety net in: every IMU read checks OPR_MODE; a reboot is logged (`imuRst`), put back in
    IMUPLUS and the heading carries on from its last value. Telemetry: `imuMode`, `imuRst`.
  - Boot-time I2C bus clear added for both buses (a sensor left mid-byte by a Teensy reset
    hung setup once inside the 8x8 init).
- [x] **Heading live check (28/9, `docs/testdata/..._heading_load_rotate.log`):** tracks spinning on
      blocks, robot turned by hand: `hdg` 0 -> 88.4 -> -3.1, `imuMode` 0x8 throughout, `imuRst` 0.
      Boot bus clear fired for real ("Wire1 was stuck - bus cleared") and it booted normally.
- [x] Lying (knocked-over) weights: notch ToF sees them (side-on 110 mm, end-on 149 mm vs empty
      196-220 mm) but **team decision: ignore them** - no code change, REJECT already backs off.
- [x] **Low box hit-then-chase bug: solved.**
- [x] **Head-on wall CAUTION veer: solved.**

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
- [x] ~~514 servo isolator board~~ **not available** (28/9). Avoid hard knocks to the crane arm; if a log ever shows an unexplained reset right after the arm is hit, that's the likely cause (back-EMF).

## 2b. Dummy rejection (inductive sensor at the notch, 40 mm up, front-on) - DONE 29/9
- [x] (done 29/9) Insert-dummy (plastic with a steel top) tested upright and lying against the
      inductive sensor.
- [x] (done 29/9) Pickup only if the inductive sensor reads metal; otherwise reverse ~10 cm and
      pivot away (option C).

## 2c. New pickup logic (inductive trigger, CREEP / REJECT): bench tests on blocks
Flash `nav`, robot on blocks, say go before pressing GO. 🤖 logs each one.
- [x] (28/9 blocks: OK, onb +1, REPOSITION) **Pickup chain:** drop a steel weight into the notch mid-round → PICKUP → crane → "OK" →
      `onb` +1 → turns away. Repeat to 3 on board → it stops collecting.
- [x] (28/9 blocks, spacer on the weight: MISS, retry, MISS, give up; + new lockout stops endless retries on a weight left in the notch) **Missed grab:** hold the weight down so the magnets can't lift it → retries once, then gives up.
- [x] (28/9 blocks: CREEP -> REJECT reverse + pivot; rear guard confirmed - hand at ~155 mm held the reverse at 0/0, then the pivot ran) **CREEP → REJECT:** hold a weight ~14 cm ahead of the bottom-left ToF, then pull it away →
      creep, no metal, reverse + pivot. Hand behind the robot during the reverse → rear guard stops it.
- [x] (28/9: stopped itself at 118.5 s, no resets, IMU 0 resets, 8x8 never stale, pickup counted) **Full 2-min round, hands off:** stops itself at ~118.5 s, no hang, no reset (`ms` never jumps
      back to 0). With a pickup in it, this also checks crane + both tracks together don't brown out.

## 2f. Motion / heading-hold PID calibration (`enctest` env) - DONE 29/9
`encoder_test.cpp` doubles as a live PID tuning rig: `kp <v>` / `ki <v>` / `kd <v>` / `imax <v>` /
`maxsteer <v>` set `headingTuning()` gains live (no reflash), `s` prints the current gains AND the
learned integral, paste-ready for `config.h` (`HEADING_KP/KI/KD/I_MAX/MAX_STEER/I_START`).
- [x] (done 29/9) Tuned KP/KI/KD on the arena floor; gains saved to `config.h`
      (`HEADING_KP=5, HEADING_KI=0.5, HEADING_KD=0.5`).
- [x] (done 29/9) Read `HEADING_I_START` off a settled `T` trace and saved to `config.h`.

## 2e. Integrate stack_test.cpp processes with old working navigation code
Combines the former "notch back-away" and "stack_test.cpp rework" items - both were about getting
the patient, discrepancy-based weight-orientation logic properly working against the real,
already-proven navigation FSM instead of tuned in isolation.
- [ ] Integrate `stack_test.cpp`'s SEARCH/CREEP/REVERSE + patient discrepancy-confirm logic into
      the main `nav` build's navigation FSM in place of the old settle-timer notch back-away check,
      then validate and tune it (live via `rev`/`turn`/`disc`/`lconf`) in a full round simulation
      before copying settled values into `config.h`.

## 2d. Third weight carried on the magnet (held at rest, not dropped)
- [x] Magnet coils **do get hot** in extended use (partner, from earlier testing), so a full-power
      hold for ~100 s isn't safe. Plan: full power to grab, then a **reduced PWM holding level**.
- [x] **Only one magnet now (team decision 28/9, reverted from the two-magnet array) - code/docs
      were still driving MAG1+MAG2 together, corrected.** Single magnet moved from pin 26 (CON74,
      no PWM) to **pin 24 (CON72)**, PWM-capable. `config.h`: `PIN_MAGNET` (was `PIN_MAG1`/`PIN_MAG2`)
      + placeholder `MAGNET_HOLD_PCT = 100`. `collection.cpp`: `analogWrite(MAGNET, ...)` in place of
      the two `digitalWrite` calls - same on/off behaviour today, PWM ready for the hold test.
      **Physically move the magnet wire from CON74 to CON72 before this is flashed to the real robot.**
      `nav` and `servotest` both build clean against the change.
- [x] **PWM hold test: done, 50% duty holds through a shake.** `MAGNET_HOLD_PCT = 50` in `config.h`.
      `magnettest` bench rig (`src/magnet_test.cpp`) retired now that it's served its purpose.
- [x] **"Pick up and hold" crane cycle for the 3rd target - implemented 28/9.** New
      `COLLECTION_HOLD` state in `collection.cpp`: on the pickup that would make `targetsOnBoard()`
      reach `MAX_TARGETS_ON_BOARD`, `navigation.cpp` calls `collection_start(true)` instead of
      `collection_start()`. The arm eases to rest with the magnet still at full power (so the swing
      doesn't drop it), then once parked the magnet drops to `MAGNET_HOLD_PCT` and **stays there
      indefinitely** - nothing in the codebase turns it off again after that (not at round end, not
      elsewhere in the FSM), only a power cycle drops it, per the team's requirement. `pickupIsThird`
      in `navigation.cpp` is decided once at `startPickup()` and held through MISS retries so a retry
      doesn't re-evaluate mid-cycle. This DOES touch `collection.cpp` (partner's file) - flagged here,
      kept small (one new state, no changes to the existing PICKUP/DROP/FINISHED states), partner
      should review before it's trusted on a real round.
- [x] **3rd-target hold confirmed on the bench (28/9), `pickuptest` env retired.** 1st/2nd weights
      triggered a normal pickup (swing to drop); the 3rd correctly parked at rest with the magnet
      held at `MAGNET_HOLD_PCT` instead of dropping, and further weights presented afterward were
      ignored (cap respected). `src/pickup_hold_test.cpp` removed now that it's served its purpose.
      Still outstanding: confirming the hold survives an actual `roundOver()` transition and a real
      MISS-then-retry on the 3rd, in a full round context rather than this isolated bench test.

## 3. On the floor  (printed parts installed first)
**Team decision 28/9: obstacle avoidance is good.** Floor testing now focuses on collecting
weights, not further avoidance tuning. `x` kills the motors.
- [x] Obstacle avoidance: tall-box run, corner handling, false weight detections near
      walls/corners, SCAN-near-wall, APPROACH "no progress" exit - all confirmed good.
- [ ] **Approach + pickup on a single weight** - the main next test.
- [x] (done 29/9) **Ground clearance.**
- [x] (done 29/9) **Pickup push-out check.**
- [x] (done 29/9) **Does the V-notch swing actually clear a rejected object?** (REJECT)
- [ ] Creep through the blind gap: tune `CREEP_SPEED_PCT` / `CREEP_MAX_MS` so a real weight reliably
      reaches the inductive sensor before REJECT fires. (Blocks 28/9: a steel weight a few mm outside
      inductive range, ~13-17 cm ahead on the bottom ToFs, was REJECTed as "no metal" - on blocks the
      creep can't push it in. Check on the floor that real weights never get rejected.)
- [ ] Tune `SIDE_NEAR_MM` (side IR, currently 150) against a red wall.
- [ ] **Raise `DRIVE_SCALE_PCT` back to 100** (`config.h`, currently 50 = half the usable pulse
      range, a safety limiter for first floor tests). Step 50 → 75 → 100 and retest at each step:
      stopping distance grows with speed, so the 8×8 thresholds may need lengthening. Time 1 m at
      50 and at 100 for the report's speed figure.
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
- [ ] **Integrate the gate (rear flap) with the rest of the system.** `gate_init()`/`gate_update()`
      run in `nav_main.cpp` (Herkulex on Serial2/CON66), but nothing ever commands it to open -
      `gate_move_test()` exists but isn't called from navigation. It's release-only (open at the
      home base to drop targets; dummies are rejected before pickup at the notch, not sorted here -
      see CLAUDE.md), so it's gated on homing/delivery existing. Also **possibly redo/reprint** the
      gate/flap mechanism itself - check with the team whether the current print still fits/works
      before wiring it in.
- [ ] **Drive encoders (27/9)**: `odometry.cpp` + `enctest` env.
  - [x] Wired: Encoder IO board on **CON55 (D2–D5)**, L A/B = 2/3, R A/B = 4/5 (one board, both motors).
        Found + fixed on the way: motor channel 1 was driving the RIGHT track (leads swapped at the
        driver), and the left motor then ran backwards (polarity flipped at its screw terminal).
  - [x] Signs (enctest, on blocks): `ENC_L_SIGN = -1`, `ENC_R_SIGN = 1` - forward counts up on both.
  - [x] `ENC_COUNTS_PER_M = 15430`: 5 GO-button runs of ~63 cm at cruise, sd 0.4% (not the arena floor).
  - [x] (done 29/9) Recheck counts/m on the **arena floor**.
  - [x] (superseded - the trim isn't the lever: encoders showed equal track speed while it still
        turned, so it's track/floor drag; the heading hold handles it) **Forward trim at cruise:** those runs turned **right ~9°** (6.8–12.2°) and drifted ~5 cm right
        over 63 cm, with `DRIVE_TRIM_L_FWD` 0.90. Part of that was the jolt from the instant stop; the
        soft stop (`DRIVE_DECEL_MS`) is now in. Rerun 3× with enctest (GO) and lower the left trim
        if it still turns right. The IMU heading hold will take out what's left.
  - [ ] Slip check (`nav`, telemetry `odo`/`odoRaw`/`slip`): drive into a wall and hold →
        `odoRaw` keeps climbing, `odo` stops, `slip` = 1. Tune `ODOM_SLIP_*` if not.
- [x] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/` (27/9; the 151 MB PDF is git-ignored, the `.md` is committed).
