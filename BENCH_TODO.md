# Bench checklist — next session with the robot

Hardware checks and changes that were queued on 2026-09-24 while the Teensy was
disconnected. Work top to bottom: the first two unblock the rest of the code.

## 1. Run the wire finder (10 min)
```
pio run -e wirefind -t upload
pio device monitor -b 115200
```
- [ ] Paste the boot report to Claude. It answers:
  - which RAW I2C bus the **XSHUT expander (CON26)** and the **ToF bus (CON35)** are cabled to
  - whether the **weight-detect ToF (CON27)** is a VL53L0X or a VL53L1X
  - whether the **8×8 (CON64)** really shows up on `Wire1` at 0x33
- [ ] Press `t` and cover each ToF in turn to confirm CON29 = bottom-left, CON28 = bottom-right,
      CON27 = weight-detect, CON30 = rear.
- [ ] Press `p`, hold a steel weight at the inductive sensor. Check pin 20 (A6Z column) reads
      **low for metal**, since `INDUCTIVE_ACTIVE_LOW = true` assumes that. The level-shift board could invert it.

## 2. Resolve the pin 28 clash (gate vs crane servo)
Pin 28 is RX7 on CON67, which now has the Herkulex level-shift board. `collection.cpp`
also drives the crane servo on pin 28.
- [ ] Find where the **crane servo signal wire** is actually plugged in.
- [ ] **Recommended:** move the Herkulex level-shift cable from CON67 to **CON66 (SERIAL2)**,
      then set `#define GATE_SERIAL Serial2` in `src/gate.cpp`. Nothing in the partner's code changes.
- [ ] Alternative: move the crane servo to **CON71 (A7Z, pin 21)** and change `BIG_SERVO`
      to 21 in `collection.cpp` and `servo_test.cpp` (tell the partner).

## 3. Drive
- [ ] Speed-ramp test. `drive()` now caps pulses at 1949 / 1051 µs (68% fwd / 73% rev of
      motor.cpp's scale). If the tracks clearly run faster with the caps raised, the driver's real
      window is wider, so raise `MOTOR_MAX_FWD_PCT` / `MOTOR_MAX_REV_PCT` in `config.h`.
- [ ] Check left/right track direction matches motor 1 = left, motor 2 = right.

## 4. Crane (with the partner)
- [ ] Watch the first few collection cycles. The per-tick `bigServo.write(1)` was removed, so the
      arm now really goes to 110° for pickup and parks at 70°. Check nothing collides.
- [ ] If the weight isn't released cleanly, raise `WEIGHTDROPDELAY` above 1000. At the moment the
      crane heads to rest straight after the magnets switch off, which is the same timing as before.
- [ ] Consider the 514 servo isolator board if the crane arm takes knocks (back-EMF can reset the CPU).

## 5. Sensors on the arena floor
- [ ] Side IR (GP2Y0A21): log readings against a red wall at 150 / 200 / 300 mm and tune
      `SIDE_NEAR_MM` (currently 150).
- [ ] 8×8: work out its orientation (which corner is row 0 / column 0) by waving a hand in front of it.

## 6. First run of the new nav build (after steps 1–2)
- [ ] Set `TOF_WIRE` / `SX_WIRE` in `config.h` from the wirefind report, and set `TOF_TYPE[2]`
      if the weight-detect ToF turns out to be an L0X.
- [ ] **Robot on blocks, tracks off the ground.** Run `pio run -e nav -t upload`. The boot log should show
      4× `ToF ... ok` and `SEN0628 8x8 ok`. The round auto-starts 3 s after boot (no GO button yet).
- [ ] Press `g` for the 8×8 grid. Set `X8_COL_SIGN` / `X8_ROW_FLIP` so a hand on the robot's left
      shows in the left columns, and pick `X8_BAND_LO/HI` rows that see walls but **not** a weight on the floor.
- [ ] Read the empty-funnel `UP` value in the telemetry and set `FUNNEL_PRESENT_MM` well below it.
- [ ] Then on the floor: obstacle avoidance, then approach + pickup on a single weight.
      The telemetry is tab-separated, so it can be pasted straight into a spreadsheet for report data.
      `x` = kill motors.

## 7. Wiring still to add
- [ ] **GO button** on a free 3-pin analogue port (e.g. A0Z, CON68). It's needed to start the round
      and zero the heading.
- [ ] Drive encoders via the encoder IO board (roadmap item 6, for homing odometry).
- [ ] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/`. CLAUDE.md refers to them but they're missing.
