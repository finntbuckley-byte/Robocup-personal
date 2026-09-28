# Robot TODO

Work top to bottom. 🤖 = Claude can do it from the laptop once the Teensy is plugged in (with a
**data** USB cable).

## 0. Next session (from 27–28/9 late)
- [x] **Notch ToF fault (28/9): the sensor (or its cable) was faulty, not the port.** Swapping
      CON27 <-> CON30 moved the fault with the sensor; a replacement on CON27 then ranged cleanly
      (all 4 ToFs stayed at 0x34-0x37, no errors). Positions re-confirmed: weight in the notch =
      59 mm (same as 25/9, so `FUNNEL_PRESENT_MM` 66 still holds), hand behind = rear 68 mm.
      **Label the faulty sensor and keep it off the robot.** TEMP diagnostics removed; `u`/`i` kept.
- [x] **Flange reprint: done.** Ruler runs / `HEADING_I_START` and `ENC_COUNTS_PER_M` are covered
      by the new §2f (motion/PID calibration) and §4 encoder recheck items - no longer blocked.
- [ ] Recalibrate `ENC_COUNTS_PER_M` **with the heading hold on**, on the **arena floor**: hold-on
      runs gave ~15,750–15,900 counts/m vs 15,640 without (steering scrub adds counts).
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

## 2b. Dummy rejection (inductive sensor at the notch, 40 mm up, front-on)
- [ ] **Needs an insert-dummy (plastic with a steel top). None found yet.** Upright in the notch
      it should read non-metal (the sensor sees the side, not the steel top). Also try it lying with
      its top facing the sensor, which may false-trigger.
- [ ] Then: pickup only if the inductive sensor reads metal; otherwise reverse about 10 cm and
      pivot away (option C). Only design the flinger if dummies keep getting re-found.

## 2c. New pickup logic (inductive trigger, CREEP / REJECT): bench tests on blocks
Flash `nav`, robot on blocks, say go before pressing GO. 🤖 logs each one.
- [x] (28/9 blocks: OK, onb +1, REPOSITION) **Pickup chain:** drop a steel weight into the notch mid-round → PICKUP → crane → "OK" →
      `onb` +1 → turns away. Repeat to 3 on board → it stops collecting.
- [x] (28/9 blocks, spacer on the weight: MISS, retry, MISS, give up; + new lockout stops endless retries on a weight left in the notch) **Missed grab:** hold the weight down so the magnets can't lift it → retries once, then gives up.
- [x] (28/9 blocks: CREEP -> REJECT reverse + pivot; rear guard confirmed - hand at ~155 mm held the reverse at 0/0, then the pivot ran) **CREEP → REJECT:** hold a weight ~14 cm ahead of the bottom-left ToF, then pull it away →
      creep, no metal, reverse + pivot. Hand behind the robot during the reverse → rear guard stops it.
- [x] (28/9: stopped itself at 118.5 s, no resets, IMU 0 resets, 8x8 never stale, pickup counted) **Full 2-min round, hands off:** stops itself at ~118.5 s, no hang, no reset (`ms` never jumps
      back to 0). With a pickup in it, this also checks crane + both tracks together don't brown out.

## 2e. Notch back-away (settle logic, `notch_backaway_test.cpp` / `notchtest` env) - DEFERRED 28/9
Team decision 28/9: stop tuning this in isolation on the bench. `UPRIGHT_BACKAWAY_SETTLE_MS` will
be calibrated during whole-round testing instead, against real funnel/CREEP approaches rather than
a hand placing objects. Bench findings kept for when this is picked back up:
- [ ] **`UPRIGHT_BACKAWAY_SETTLE_MS`**: the anchor/window settle check (not frame-to-frame) is
      sensitive to how slowly an object is placed - a hand placing a weight naturally decelerates on
      approach, and at 400ms the trigger still fired ~400-600ms before the inductive sensor caught up
      on a slow creep-in. 800ms passed that case; `config.h` currently has 600ms, untested. Confirm/
      retune against real robot approaches once whole-round testing is underway, not on the bench.
- [ ] **Define proper "ignore this object" behaviour for anything confirmed not upright metal**
      (dummy, lying weight, or anything else the back-away/REJECT path backs off from), not just the
      existing 4 s `REJECT_SUPPRESS_MS` spot-ignore. Right now a rejected object can be re-found and
      re-approached repeatedly once the 4 s window lapses, wasting round time on the same dummy. Needs
      a design decision: how "confirmed non-target" is remembered (position via odometry/IMU, since
      there's no colour/ID to recognise it by), how long/whether it's ignored for the rest of the
      round vs. just longer than 4 s, and how that interacts with a real target weight later ending up
      near the same spot.

## 2f. Motion / heading-hold PID calibration (`enctest` env, extended 28/9)
`encoder_test.cpp` now doubles as a live PID tuning rig, not just encoder bring-up: `kp <v>` /
`ki <v>` / `kd <v>` / `imax <v>` / `maxsteer <v>` set `headingTuning()` gains live (no reflash),
`s` prints the current gains AND the learned integral, paste-ready for `config.h`
(`HEADING_KP/KI/KD/I_MAX/MAX_STEER/I_START`). `T` still prints the per-run heading/steer/integral
trace. Recommended order (also in the file header): KP alone until it weaves, add KD to damp
overshoot, add KI last to kill steady-state drift, then read the settled integral into
`HEADING_I_START`.
- [ ] **Tune KP/KI/KD on the arena floor** using `enctest`'s marked-distance runs, `H` to toggle
      hold on/off for comparison. 3-5 repeats per gain change (single lucky/unlucky run isn't
      enough), log both final heading error (`hdg`) and mid-run wobble from the `T` trace.
- [ ] **Read `HEADING_I_START`** off a settled `T` trace once KP/KI/KD are decided, and put it in
      `config.h` so rounds start already compensated for the track/floor drag bias instead of
      drifting for the first second while KI winds up from zero.
- [ ] Blocked on the flange reprint per §0 before the ruler runs are meaningful.

## 2g. Top/bottom baseplate ToF - lying-weight reject (draft code in, hardware not fitted)
New 5th ToF (`TOF_TOP`, proposed CON31/XSHUT4, VL53L0X, addr 0x38) pairs with the existing notch
ToF (`TOF_UPRIGHT`) as "bottom": bottom sees something + top doesn't (bench-confirmed blind to a
weight on its side) = reject before creeping it in. Replaces the old inductive-gated,
settle-timer back-away check, which was rejecting real upright weights too fast with no top
sensor to confirm against. Code (`navigation.cpp` `lyingWeightConfirmed()`, `config.h`
`BOTTOM_PRESENT_MM`/`TOP_PRESENT_MM`/`LYING_CONFIRM_MS`) is drafted but **not tested on any
hardware** - the top sensor isn't wired yet (in progress 28/9).
- [ ] Wire CON31 to the new VL53L0X, run `wirefind` to confirm it answers on XSHUT4 and pick up
      an address - check it lands where `TOF_ADDRESS_START + 4` (0x38) expects.
- [ ] Confirm mounting height above the baseplate matches what's needed to stay blind to a lying
      weight while still seeing an upright one - this was bench-tested on the bench rig, not yet
      on the actual mounted sensor.
- [ ] **Bench-characterise both thresholds** the same way `FUNNEL_PRESENT_MM` was: log raw mm for
      steel upright, plastic upright, a lying weight, and empty, for BOTH `tofUpright` and
      `tofTop`. Set `BOTTOM_PRESENT_MM`/`TOP_PRESENT_MM` from real data - both are placeholders
      right now (160mm / 80mm).
- [ ] Confirm `LYING_CONFIRM_MS` (100ms) doesn't false-reject a real weight still sliding into
      place, the same failure mode the old check had - watch for this specifically on the first
      full-round test with the new sensor wired in.
- [ ] Flash `nav`, robot on blocks: place a lying weight in the notch mid-round and confirm REJECT
      fires quickly (before CREEP would have timed out); place a real upright weight and confirm
      it does NOT reject.

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
- [ ] **Ground clearance - filing scheduled tonight (28/9).** Only blocks base-rim-dependent
      tests (homing/delivery: crossing the rim both ways, the 25 mm speed bump, the 100 mm/30%
      ramp). Everything else in this section - approach/pickup, REJECT, CREEP, arena sensor check,
      side IR, speed ramp-up - can be tested now, filed or not.
- [ ] **Pickup push-out check (arena):** every PICKUP event line now says `metal left at X.XX s`
      (data only). Real lifts on blocks 28/9 lost the metal ~1.2 s in (the lift). A weight pushed
      out by the arm would leave earlier and still be counted as collected (false `onb` -> the
      3-target cap stops collecting early). From the arena logs: if push-outs happen, add the
      crane "lift started" check (option 3: small getter in collection.cpp, partner's OK) and
      only count a pickup if metal was still there when the lift began.
- [ ] **Does the V-notch swing actually clear a rejected object?** (REJECT = reverse ~10 cm, pivot,
      ignore the spot 4 s.) The team isn't convinced it works, so test it with a plastic dummy and a
      knocked-over weight. Tune `REJECT_REVERSE_MS` / pivot time / `REJECT_SUPPRESS_MS`. If objects
      don't get pushed clear, or keep getting re-found: longer reverse + bigger turn first, then
      the under-plate flinger.
- [ ] Creep through the blind gap: tune `CREEP_SPEED_PCT` / `CREEP_MAX_MS` so a real weight reliably
      reaches the inductive sensor before REJECT fires. (Blocks 28/9: a steel weight a few mm outside
      inductive range, ~13-17 cm ahead on the bottom ToFs, was REJECTed as "no metal" - on blocks the
      creep can't push it in. Check on the floor that real weights never get rejected.)
- [ ] **Test in the actual arena where possible:** black floor/walls change what the bottom ToFs, the
      8x8 floor rows (r5-r7 set on a lighter floor) and the side IR see. Re-run `g` there first.
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
  - [ ] Recheck counts/m on the **arena floor** (1–2 runs) when it's free.
  - [x] (superseded - the trim isn't the lever: encoders showed equal track speed while it still
        turned, so it's track/floor drag; the heading hold handles it) **Forward trim at cruise:** those runs turned **right ~9°** (6.8–12.2°) and drifted ~5 cm right
        over 63 cm, with `DRIVE_TRIM_L_FWD` 0.90. Part of that was the jolt from the instant stop; the
        soft stop (`DRIVE_DECEL_MS`) is now in. Rerun 3× with enctest (GO) and lower the left trim
        if it still turns right. The IMU heading hold will take out what's left.
  - [ ] Slip check (`nav`, telemetry `odo`/`odoRaw`/`slip`): drive into a wall and hold →
        `odoRaw` keeps climbing, `odo` stops, `slip` = 1. Tune `ODOM_SLIP_*` if not.
- [x] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/` (27/9; the 151 MB PDF is git-ignored, the `.md` is committed).
