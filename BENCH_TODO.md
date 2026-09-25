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
- [ ] 🤖 **First nav run**: `pio run -e nav -t upload`. Check all 4 ToFs and the 8×8 report `ok`.
- [ ] 🤖 Press `g` to print the 8×8 grid, then set its orientation (`X8_COL_SIGN`, `X8_ROW_FLIP`) and
      obstacle rows (`X8_BAND_LO/HI`), which must see walls but not floor weights.
- [ ] 🤖 Read the empty-funnel `UP` value in the telemetry and set `FUNNEL_PRESENT_MM` below it.
- [x] Speed-ramp test: check the 68% fwd / 73% rev pulse caps. (25/9: team confirmed max 1950 fwd / 1050 rev µs, caps match) Raise them in `config.h` if the tracks
      clearly run faster above that.
- [ ] Check the tracks turn the right way: motor 1 = left, motor 2 = right.

## 2. Crane (with the partner)
- [ ] Watch the first cycles: the arm should now really go to 110° for pickup and park at 70°.
- [ ] If weights don't release cleanly, raise `WEIGHTDROPDELAY` above 1000.
- [ ] Add the 514 servo isolator board if the arm takes knocks (a knock can reset the CPU).

## 3. On the floor
- [ ] Obstacle avoidance on its own. `x` kills the motors.
- [ ] Approach + pickup on a single weight.
- [ ] Tune `SIDE_NEAR_MM` (side IR, currently 150) against a red wall.
- [ ] Paste the telemetry into a spreadsheet for report data: sorting/collection accuracy, speed,
      avoidance success rate.

## 4. Later
- [ ] Wire the **GO button** (e.g. A0Z, CON68) and set `PIN_GO`.
- [ ] Fit the IMU and write homing + delivery. Then turn on `USE_HOMING`.
- [ ] Gate/flap sorting logic: keep metal, drop dummies.
- [ ] Drive encoders via the encoder IO board, for homing odometry.
- [ ] Add `Parts_Summary_2026B.pdf` / `.md` to `docs/`.
