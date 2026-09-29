#pragma once
// ============================================================================
//  config.h  -  full RoboCup navigation + collection build. ALL pins/tuning
//  here.
//
//  HARDWARE:
//    Drive:  2x DC motor via DFR0513 PPM driver (motor.cpp/h, PARTNER'S CODE
//            - unchanged). Motor 1 = LEFT (pin 0), Motor 2 = RIGHT (pin 1).
//    Pickup: swing-arm crane + 2 electromagnets (collection.cpp/h, PARTNER'S
//            CODE - minimally modified, exposes collection_busy()).
//    Sensing (connector map from the physical board, 2026-09-24):
//      5x ToF via SX1509 XSHUT lines (ToF connectors CON27-34 = XSHUT0-7):
//        bottom-left  VL53L0X  CON29  - weight detection / APPROACH steering
//        bottom-right VL53L0X  CON28  - weight detection / APPROACH steering
//        weight-detect (upright) CON27 - "bottom" of the lying-weight check
//        rear         VL53L0X  CON30  - reversing clearance (2026-09-28: was L1X)
//        baseplate-top VL53L1X CON31  - "top" of the lying-weight check
//                                       (2026-09-28: NOT YET FITTED, wiring
//                                       in progress - see config.h TOF_TOP)
//      1x SEN0628 8x8 ToF, front, CON64 (RAW I2C1 = Wire1) - obstacles
//      2x analog IR (GP2Y0A21, white), side-facing: left CON24 (A9Z),
//        right CON23 (A8Z)
//      1x inductive proximity sensor at the end of the funnel, via the
//        inductive level-shift board to CON70 (A6Z, pin 20) - metal
//        (real weight) vs non-metal (dummy) discrimination
//    Not fitted: IMU (left out for now), TCS34725 colour sensor (dropped).
//    The old top-front + corner ToFs are gone - the SEN0628 does their job.
//
//  BUILDS: [env:nav] = full navigation (nav_main.cpp), the default env.
//
//  ARCHITECTURE: same DISCRETE (bang-bang+hysteresis) / CONTINUOUS
//  (proportional/PD) split used throughout this project. See navigation.cpp
//  for the state machine, funnel_sensor.h for the sorting logic and round.h
//  for the round timer / target cap.
//
//  PICKUP: APPROACH PD-steers onto a weight candidate using the bottom ToF
//  pair, then the robot STOPS, calls collection_start(), and waits on
//  collection_busy() for the crane to finish before moving on.
//
//  FUTURE HOOKS (not implemented - no hardware specifics given yet):
//    IMU        would replace the timed SCAN spin and REPOSITION turn with
//               heading control, and drive RETURN_HOME. Search for "IMU HOOK"
//               in navigation.cpp.
//    IR beacon  fallback homing. Search for "BEACON HOOK" in navigation.cpp.
//
//  PIN CONFIDENCE - read this before flashing:
//    CONFIRMED   motor pins (motor.cpp), collection pins (collection.cpp),
//                the original 4 ToF XSHUT lines, SEN0628 port, side IR pins
//                + L/R + part, inductive pin, gate on CON67 (connector map,
//                2026-09-24).
//    TODO(verify) ToF model on the weight-detect port; which RAW bus the ToF
//                bus (CON35) and XSHUT expander (CON26) are cabled to - the
//                wirefind report answers both.
//    UNCONFIRMED TOF_TOP (CON31/XSHUT4) - not physically wired yet
//                (2026-09-28). Pin/address assignment is a proposal only;
//                confirm with wirefind once it's connected, and bench-verify
//                LYING_NOTCH_MAX_MM/LYING_TOP_MAX_MM/LYING_CONFIRM_MS below
//                before trusting them on the robot.
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// MODULE ENABLE SWITCHES
// ---------------------------------------------------------------------------
#define USE_REAR_TOF    1
#define USE_SIDE_IR     1
#define USE_FUNNEL_SORT 1     // funnel ToF + inductive classification
#define USE_SCAN        1
// RE-ENABLED 2026-09-29: was disabled 2026-09-28 (arena testing found the
// original top/bottom lying-weight reject wasn't working). navigation.cpp
// lyingWeightConfirmed() has since been rewritten around the patient,
// discrepancy-based logic proven out in stack_test.cpp/BENCH_TODO.md 2h -
// integrating it into the nav build for a full round simulation test.
#define USE_LYING_WEIGHT_REJECT 1
// 30 Sept round adjustment: keep notch-assisted creep, but disable the
// mismatch rejection that has been abandoning upright weights.
const bool REJECT_FROM_TOP_NOTCH_MISMATCH = false;
// Disable backing-up in both weight rejection and obstacle escape.
// Pivot turns still use opposite track directions.
const bool NAV_RECOVERY_REVERSE_ENABLED = false;

// ---------------------------------------------------------------------------
// ROBOT GEOMETRY  (mm)
// ---------------------------------------------------------------------------
const int ROBOT_WIDTH_MM   = 165;
const int ROBOT_HEIGHT_MM  = 170;
const int WEIGHT_HEIGHT_MM = 70;

// ---------------------------------------------------------------------------
// ToF CHAIN  -  5 sensors, XSHUT via SX1509 @ 0x3F
// ---------------------------------------------------------------------------
const byte SX1509_ADDRESS = 0x3F;
// re-addressed from 0x34 upward in index order - 0x30 would put index 3 on
// 0x33, the SEN0628's fixed address
#define TOF_ADDRESS_START 0x34

// index order used EVERYWHERE:
const int TOF_BL      = 0;   // bottom-left  VL53L0X          CON29 / XSHUT2
const int TOF_BR      = 1;   // bottom-right VL53L0X          CON28 / XSHUT1
// Confirmed 30 Sept: physical right points left and physical left points right.
const bool LOWER_BEAMS_CROSSED = true;
const int TOF_UPRIGHT = 2;   // weight-detect, across notch   CON27 / XSHUT0
const int TOF_REAR    = 3;   // rear VL53L0X (2026-09-28: swapped from L1X)  CON30 / XSHUT3
// baseplate-top, lying-weight reject   CON31 / XSHUT4 - NOT YET FITTED
// (2026-09-28, proposed): bench-confirmed blind to a weight lying on its
// side, unlike TOF_UPRIGHT which still sees it. See navigation.cpp
// lyingWeightConfirmed(). Address 0x38 (TOF_ADDRESS_START + 4) is clear of
// the OLED (0x3C) and both SX1509s (0x3E/0x3F). Team confirmed 2026-09-28
// this is a VL53L1X (both "weight discrimination" sensors - this and
// TOF_UPRIGHT - are L1X; the rear/reversing sensor is now the L0X instead).
const int TOF_TOP     = 4;
const int TOF_COUNT   = 5;

// SX1509 XSHUT line for each index (connector CONn = XSHUT(n-27))
const int XSHUT_TOF[TOF_COUNT] = { 2, 1, 0, 3, 4 };

// sensor model at each index: 0 = VL53L0X (short), 1 = VL53L1X (long)
// 2026-09-28: rear and top-baseplate swapped from the original assumption -
// rear is now the VL53L0X, top-baseplate (TOF_TOP) is a VL53L1X alongside
// the notch sensor (both are the "weight discrimination" pair).
const int TOF_TYPE[TOF_COUNT] = { 0, 0, 1, 0, 1 };

// VL53L1X distance mode per index (ignored for L0X): true = Short (to ~1.3m,
// better in ambient light), false = Long (to ~4m). The notch and top
// sensors only look tens of mm, so both run Short; rear is now L0X so its
// entry here is unused.
const bool TOF_SHORT_MODE[TOF_COUNT] = { false, false, true, false, true };

// Which RAW I2C bus each sub-assembly is cabled to. Only the ToFs (via I2C In
// CON35) and the XSHUT expander (via I2C In CON26) matter here.
// TODO(verify): run wirefind - its report names the bus for both. RAW I2C0 =
// Wire, RAW I2C1 = Wire1.
// Teensy 4.0 pins of the two RAW I2C buses - used by the boot-time bus clear
const uint8_t I2C0_SDA_PIN = 18, I2C0_SCL_PIN = 19;   // Wire
const uint8_t I2C1_SDA_PIN = 17, I2C1_SCL_PIN = 16;   // Wire1

#define TOF_WIRE Wire
#define SX_WIRE  Wire

// SX1509 registers (bank A = I/O 0-7 = XSHUT0-7)
const byte SX_REG_DIR_A  = 0x0F;
const byte SX_REG_DATA_A = 0x11;

const unsigned long TOF_BOOT_MS      = 50;   // after raising XSHUT - proven necessary
const unsigned long TOF_PERIOD_MS    = 50;   // continuous-mode inter-measurement period
const unsigned long TOF_L1X_BUDGET_US = 33000;
const uint16_t TOF_MAX_VALID_MM = 4000;   // VL53L1X long-mode max; above = garbage

// A ToF with no new data for this long is STALE (reads 0, tofOk() false) and
// gets an XSHUT reset + re-init (tof.cpp recoverSensor). Sensors range every
// TOF_PERIOD_MS (50), so 500 = ~10 missed readings. Recovery blocks ~60-150ms,
// so retries are rate-limited, backing off after 3 failures in a row.
// (28/9: bad connection at the notch sensor's end - it dropped off the bus
// and rebooted to 0x29 whenever the cable moved.)
const unsigned long TOF_STALE_MS            = 500;
const unsigned long TOF_RESET_LOW_MS        = 10;   // XSHUT held low for a reset (same as boot)
const unsigned long TOF_RECOVER_INTERVAL_MS = 2000;
const unsigned long TOF_RECOVER_BACKOFF_MS  = 10000;

// ---------------------------------------------------------------------------
// SEN0628 8x8 MATRIX ToF  -  front obstacle sensing
// ---------------------------------------------------------------------------
// CON64 = RAW I2C1 per nav_test's board notes (CON62-64 = I2C1).
// TODO(verify): wirefind report shows 0x33 on Wire1, not Wire.
#define X8_WIRE    Wire1
const byte X8_ADDRESS = 0x33;

// Grid is row-major (buf[row*8 + col], per the DFRobot example), in mm.
// Orientation depends on how it's mounted: print the grid ('g' in the nav
// build's serial menu), wave a hand top-left of the sensor, flip these until
// the grid matches reality. 30 Sept captures support the unflipped mapping:
// low floor/weight returns at r7; left/right targets at low/high columns.
const float X8_COL_SIGN   = +1.0f;   // +1: column 7 is the robot's RIGHT
#define     X8_ROW_FLIP   0          // 1: row 0 is the BOTTOM of the field
const float X8_FOV_DEG    = 60.0f;   // horizontal field of view
// Rows used for OBSTACLES. They must look ABOVE weight height (70mm) so a
// weight on the floor isn't treated as a wall - pick from the grid print.
// Measured 2026-09-25 facing open floor: r7/r6/r5 see the FLOOR at
// ~390/490/640mm (uniform across columns - also fakes "wall ahead"), so r5
// is out. An upright weight 25cm ahead is below r7 entirely (the bottom ToFs
// see it, the 8x8 doesn't). Orientation confirmed: hand on the left -> c0-c3,
// r0 = top (sees over the 400mm wall).
const uint8_t X8_BAND_LO  = 2;
const uint8_t X8_BAND_HI  = 4;
const uint16_t X8_MAX_VALID_MM   = 3000;  // beyond this = no return
const uint16_t X8_WALL_SPREAD_MM = 150;   // flat across >=7 columns = wall

// getAllData() blocks ~20-40ms per call (the library polls with delay(17)),
// so don't read faster than needed. setRangingMode() blocks 5s at init.
const unsigned long X8_READ_MS  = 100;
// "0" is ambiguous (nothing in range OR too close). A half that read closer
// than X8_CLOSE_LATCH_MM and then 0 is treated as still that close for
// X8_ZERO_HOLD_MS (x8.cpp latched()). Tall-box test 28/9: 33 -> 0 -> 53 mm.
const uint16_t X8_CLOSE_LATCH_MM = 150;
const unsigned long X8_ZERO_HOLD_MS = 600;
const int X8_MODE_RETRIES = 3;
// No fresh frame for this long -> treat the front as unknown (crawl).
const unsigned long X8_STALE_MS = 400;

// ---------------------------------------------------------------------------
// ANALOG IR SENSORS  -  2x side, wall/scrape nudge
// ---------------------------------------------------------------------------
// Side L/R mapped on the physical board: left = CON24 (A9Z, pin 23),
// right = CON23 (A8Z, pin 22). Confirmed by hand test 2026-09-25: object at
// ~150mm on the left read 2100 counts on pin 23 = 147mm via the curve below.
const int PIN_IR_SIDE_L = A9;      // CON24
const int PIN_IR_SIDE_R = A8;      // CON23

const float IR_ADC_VREF   = 3.3f;   // Teensy analog reference
const float IR_ADC_COUNTS = 1023.0f;// default 10-bit analogRead
const int   IR_SAMPLES    = 5;      // simple averaging - these are noisy

// Side IR are WHITE = GP2Y0A21, 100-800mm. Published fit is
// cm = 27.728 * V^-1.2045, so mm = 277.28 * V^-1.2045.
// Below ~100mm the output FALLS again, so a wall closer than that can read
// as far away - keep SIDE_NEAR_MM well above the 100mm floor.
const float IR_SIDE_A = 277.28f;
const float IR_SIDE_B = -1.2045f;
const int   IR_SIDE_MIN_MM = 100;
const int   IR_SIDE_MAX_MM = 800;

const int SIDE_NEAR_MM   = 150;     // was 80 - below this part's range. TODO(verify): tune on the arena
const int SIDE_NUDGE_PCT = 15;

// ---------------------------------------------------------------------------
// FUNNEL PRESENCE  -  the weight-detect ToF (TOF_UPRIGHT, CON27) across the
// notch. Something inside this band = an object is in the funnel.
// Measured 2026-09-25: empty notch 77-112mm (and up to 1.5m looking out),
// steel upright 57-61mm, plain plastic dummy 70-74mm, steel lying = not seen.
// 66 cleanly separates empty from steel; plastic may read "absent", which is
// fine - presence is telemetry now, the pickup trigger is the inductive.
// ---------------------------------------------------------------------------
const int FUNNEL_MIN_MM     = 20;   // below this = sensor noise / blind zone
const int FUNNEL_PRESENT_MM = 66;
// Back away if the notch ToF sees an object very close. Trigger is an inclusive
// distance band; re-arm only after the reading clears this hysteresis distance.
// A weight sliding in toward pickup passes through this same band while still
// closing, so a snapshot distance check can't tell it apart from a dummy sitting
// still - it must also be SETTLED (not still getting closer) before judging.
// TODO(verify): widened from 30-50/clear 60 on 2026-09-28 bench findings (real
// approaches read up to ~70mm); retune settle tolerance/time on the bench.
const int UPRIGHT_BACKAWAY_MIN_MM = 30;
const int UPRIGHT_BACKAWAY_MAX_MM = 80;
const int UPRIGHT_BACKAWAY_CLEAR_MM = 90;
const int UPRIGHT_BACKAWAY_SETTLE_TOL_MM = 4;      // max drift from the settle-window anchor to still count as "settled"
// TODO(verify): 400ms halves the prior 800ms bench value (2026-09-28) now that
// the settle check is anchor/window-based, not frame-to-frame - the anchor
// approach only needs (approach speed x window) to exceed SETTLE_TOL_MM to
// keep rejecting a moving weight, and the bench-observed slow creep (~30-40
// mm/s) clears that at well under 400ms. Retest against the slowest deliberate
// placement speed before going lower.
const unsigned long UPRIGHT_BACKAWAY_SETTLE_MS = 800;   // must stay settled this long before judging
const unsigned long UPRIGHT_BACKAWAY_CONFIRM_MS = 100;  // extra debounce once settled+no-metal is seen
// The distance settle check and the inductive reading are debounced separately:
// a single noisy LOW sample from the inductive sensor must not veto a weight
// that has otherwise read metal steadily (bench-confirmed 2026-09-28: a settled
// steel weight reading metal=1 for >1s still fired a false back-away on one
// stray 0 sample without this).
const unsigned long UPRIGHT_BACKAWAY_METAL_ABSENT_MS = 150;

// ---------------------------------------------------------------------------
// INDUCTIVE SENSOR  -  metal (real weight) vs non-metal (dummy) at the
// funnel end. LJ18A3-8-Z/BY -> inductive interface/level-shift board ->
// CON70 (A6Z) = pin 20, read as digital. Active LOW matches
// the partner's original tested METALLIC = 0.
// ---------------------------------------------------------------------------
const int  PIN_INDUCTIVE        = 20;     // CON70 (A6Z)
const bool INDUCTIVE_ACTIVE_LOW = true;

// how long after the funnel ToF first sees something to keep sampling the
// inductive sensor before declaring a verdict (metal can trigger briefly
// mid-slide, not necessarily at the exact instant presence trips)
const unsigned long SORT_CLASSIFY_WINDOW_MS = 200;

// ---------------------------------------------------------------------------
// OBSTACLE AVOIDANCE  (SEN0628 left/right halves; bottom pair is weights only)
// ---------------------------------------------------------------------------
const int NEAR_MM       = 250;
const int FAR_MM        = 500;
const int CAUTION_MM    = 430;
const float KC_AVOID    = 0.11f;
const int CAUTION_VEER_MAX = 22;
const int REAR_STOP_MM   = 250;   // rear ToF: stop reversing inside this
const int BLIND_SPEED_PCT = 30;   // cruise speed while the 8x8 is stale/missing

const unsigned long MIN_TURN_MS    = 700;
const unsigned long MAX_TURN_MS    = 2500;
const unsigned long FLIP_WINDOW_MS = 2000;
const int  FLIPS_BEFORE_ESCAPE     = 2;
const unsigned long ESCAPE_REV_MS  = 450;
const unsigned long ESCAPE_SPIN_MS = 900;

// ---------------------------------------------------------------------------
// SPEEDS (%)
// ---------------------------------------------------------------------------
const int CRUISE_SPEED_PCT   = 100;
const int TURN_SPEED_PCT     = 100;
const int APPROACH_SPEED_PCT = 55;
const int SCAN_SPIN_PCT      = 50;
const int REPOSITION_SPEED_PCT = 55;
const unsigned long REPOSITION_TURN_MS = 700;

// DFR0513 max pulses confirmed by the team 2026-09-25: 1950us forward,
// 1050us reverse. motor.cpp now maps 100% to exactly those limits itself
// (FULL_FORWARD 1950 / FULL_BACKWARD 1050), so no extra cap is needed here.
// Was 68/73 when motor.cpp mapped 100% to 2.0/1.0 ms; keeping those with the
// new motor.cpp double-limited top speed to ~1915/1088 us.
const int MOTOR_MAX_FWD_PCT = 100;
const int MOTOR_MAX_REV_PCT = 100;

// Scales EVERY drive command (after navigation, before the pulse caps).
// User-requested 90% drive scale, 30 Sept; prior 50% version saved as
// WORKING BOT GO GO GO. This also scales approach, creep and pivot commands.
const int DRIVE_SCALE_PCT = 90;

// Per-track, per-direction trims: multiply that track's percent (after
// scaling) so the robot drives straight open-loop. Measured on the ground
// 2026-09-27, one speed each:
//   forward: left x0.90 -> ~1 cm deviation over 1 m
//   reverse: right x0.98 -> "mostly straight"
// TODO(verify): measured with the OLD motor.cpp minimums (1842/1188 us);
// MIN_FORWARD/MIN_BACKWARD are now 1730/1350, so recheck both. The IMU
// heading hold will trim out what these don't.
// 0.90 still turned RIGHT ~10 deg over 65 cm at cruise (soft stop, 3 runs),
// so 0.86 is being tried. Resolution: drive() rounds to whole percent, so at
// DRIVE_SCALE_PCT 50 each ~0.02 of trim = 1% = ~2 us of pulse (0.86 -> 43%,
// 0.88 -> 44%, 0.90 -> 45%).
const float DRIVE_TRIM_L_FWD = 0.86f;
const float DRIVE_TRIM_R_FWD = 1.00f;
const float DRIVE_TRIM_L_REV = 1.00f;
const float DRIVE_TRIM_R_REV = 0.98f;

// Soft start / soft stop: time for a track to ramp 0 -> 100% (speeding up)
// and 100% -> 0 (slowing down). A direction change ramps down to 0 first,
// then up the other way. The soft stop was added 2026-09-27: instant stops
// jolted the robot and knocked its heading off at the end of every run.
// driveHardStop() skips the ramp - used for the 'x' kill and the rear guard.
// NOTE: the soft stop adds stopping distance (~2 cm at DRIVE_SCALE_PCT 50,
// more when that goes up), and timed moves (REJECT reverse, REPOSITION turn)
// change slightly - retune those on the floor.
const unsigned long DRIVE_RAMP_MS  = 300;
const unsigned long DRIVE_DECEL_MS = 200;

// ---------------------------------------------------------------------------
// ODOMETRY  -  drive-motor encoders via the Encoder IO board (510)
// ---------------------------------------------------------------------------
// Distance travelled for homing. Heading comes from the IMU, NOT from the
// encoders (tracks skid in every turn).
// TODO(verify): pins. The parts summary says a DIGITAL port carries 4 lines,
// so one 8-pin cable should carry both encoders' A/B. Assumed DIGITAL RAW2
// (CON55) = D2-D5. Check with `pio run -e enctest -t upload`: turn each
// track by hand and make sure the right column counts.
const int PIN_ENC_L_A = 2;
const int PIN_ENC_L_B = 3;
const int PIN_ENC_R_A = 4;
const int PIN_ENC_R_B = 5;
// +1/-1 so driving FORWARD counts UP on both tracks. The motors are mirrored,
// so one side probably needs -1. TODO(verify) with enctest.
const int ENC_L_SIGN = -1;   // confirmed 2026-09-27 (enctest, after fixing the left motor polarity)
const int ENC_R_SIGN = 1;    // confirmed 2026-09-27
// Encoder counts (4x quadrature) per metre of travel (mean of both tracks).
// Measured 2026-09-27 with enctest at cruise (100% x DRIVE_SCALE_PCT 50):
//   hard stop, 5 runs ~63 cm: 15386-15535 (mean 15430) - robot slid a bit
//     after the tracks stopped, so distance without counts -> reads low
//   soft stop, 3 runs ~65 cm: 15517-15755 (mean 15636)  <- used
// Not the arena floor (it was busy) - recheck there if distances look off.
const float ENC_COUNTS_PER_M = 15640.0f;

// Slip catching. While driving straight, the encoder travel is checked every
// ODOM_SLIP_WINDOW_MS against how much closer the thing ahead (8x8) - or
// behind when reversing (rear ToF) - actually got. Tracks spinning against a
// wall / the other robot show encoder travel with little closure: that
// window counts only the ToF closure instead. Turns aren't checked (heading
// is the IMU's job; forward distance during a spin is ~0 anyway).
// KNOWN LIMIT: something ahead that moves away (the other robot) looks like
// slip, so distance is under-counted then - the safe direction for homing.
const unsigned long ODOM_SLIP_WINDOW_MS   = 300;
const float    ODOM_SLIP_MIN_TRAVEL_MM    = 40.0f;  // encoders must claim this much before judging
const float    ODOM_SLIP_RATIO            = 0.5f;   // closure < ratio x encoder travel = slipping
const uint16_t ODOM_SLIP_MAX_RANGE_MM     = 1200;   // only trust closure on something this close
const int      ODOM_STRAIGHT_TOL_PCT      = 15;     // |cmdL - cmdR| within this = straight
// Stall: commanded to move but the encoders haven't changed for this long
// (tracks jammed, or motor/encoder fault). Reported only - nav doesn't act on it yet.
const unsigned long ODOM_STALL_MS         = 400;
const int      ODOM_STALL_MIN_PCT         = 20;

// ---------------------------------------------------------------------------
// IMU  -  BNO055 (SEN0253), heading only
// ---------------------------------------------------------------------------
// CON61 = RAW I2C0 -> Wire (fitted 2026-09-27, mid-robot, flat, board X arrow
// pointing BACKWARDS - irrelevant for heading, which is rotation about the
// vertical axis). Run in IMUPLUS (gyro + accel, no magnetometer: the motors
// and electromagnets would corrupt it). Heading is relative - zeroed at GO.
#define IMU_WIRE Wire
const uint8_t IMU_ADDR = 0x28;          // BNO055 default (0x29 if its ADR pin is high)
// Signs so that in firmware + = turned RIGHT (clockwise seen from above).
// imutest 2026-09-27, unambiguous check (standing behind the robot, front
// swung to the RIGHT ~33 deg): raw BNO heading went UP, raw gyro Z went
// NEGATIVE. So heading +1, gyro rate -1. (An earlier by-hand test was
// misdescribed and briefly set the heading sign to -1 - that made the hold
// steer INTO the error on the blocks check.)
const int IMU_HEADING_SIGN = +1;
const int IMU_GYRO_SIGN    = -1;
const unsigned long IMU_READ_MS = 20;   // 50 Hz
// after a detected BNO055 reboot, hold the last heading this long while it
// switches back to IMUPLUS and the fusion restarts (datasheet: ~7ms mode
// switch; the extra margin lets the first fused samples settle)
const unsigned long IMU_RECOVER_MS = 100;

// Heading hold (imu.cpp headingHoldSteer): PID,
//   steer % = KP*err + KI*integral(err) - KD*rate
// used as drive(speed + steer, speed - steer).
// 28/9 floor runs, PD only (KP 2, KD 0.2): the ~10 deg right turn became a
// steady ~5 deg (IMU matched the ruler within 0.7 deg) - P alone settles
// where KP*err balances the track drag. KI added to remove that offset.
// TODO(verify): CHOSEN AS A STARTING POINT (2026-09-28) from enctest floor
// traces, not fully validated - the traces so far showed clean tracking of
// small errors but hadn't fully converged after a mid-run disturbance within
// a 3s test window. Keep tuning with enctest ('H' toggles the hold, 'T'
// prints traces): weaving = lower KP/KI or raise KD; slow to come back =
// raise KI.
const float HEADING_KP        = 5.0f;   // % per degree of error
const float HEADING_KI        = 0.5f;   // % per degree-second of error
const float HEADING_KD        = 0.5f;   // % per deg/s of yaw rate (damping)
const float HEADING_I_MAX     = 20.0f;  // % cap on the integral's share (anti-windup)
// Where the integral starts at every headingHoldReset() (boot, round start):
// the robot's learned drag bias, so each round begins already compensated.
// TODO(verify): 0 until measured - read the steady 'integral' column from
// enctest traces ('T') and put it here once it settles to a stable value.
const float HEADING_I_START   = 0.0f;
const int   HEADING_MAX_STEER = 30;     // % cap on the total
// nav FORWARD: hold the heading while cruising. Any intentional steer (8x8
// veer, side-IR nudge) re-aims the hold at the current heading, and for the
// first HEADING_HOLD_SETTLE_MS after entering FORWARD it just tracks (the
// soft stop means a turn is still finishing).
const bool  USE_HEADING_HOLD        = true;
const unsigned long HEADING_HOLD_SETTLE_MS = 300;

// ---------------------------------------------------------------------------
// WEIGHT DETECTION  (bottom ToF vs 8x8 obstacle band, same side)
// ---------------------------------------------------------------------------
// A bottom ToF return with nothing at a similar distance in the 8x8's
// obstacle band on that side = something short = weight candidate.
// KNOWN LIMITATION: a weight within DIFF_CLEAR_MARGIN_MM of a wall reads as
// wall, so weights against walls are ignored.
const int WEIGHT_MIN_MM        = 60;
// Tall-box test 28/9: the false candidates were at 510-675mm - the bottom
// ToFs point outward, past the 8x8's field of view, so a side wall at an
// angle reads as "low + nothing above it" = weight. Real weights were seen
// from ~140mm. The robot cruises closer anyway, so a shorter range costs little.
const int WEIGHT_MAX_MM        = 500;   // was 700
const int DIFF_CLEAR_MARGIN_MM = 250;
const int WEIGHT_STICK_MS      = 250;   // was 120 - single-frame flickers started approaches

// APPROACH must make progress: the candidate has to get at least
// APPROACH_MIN_CLOSE_MM closer every APPROACH_PROGRESS_MS, and the tracks
// mustn't be stalled - otherwise drop it and suppress it. Catches walls seen
// at an angle (their distance grew 411 -> 713mm while "approaching") and a
// robot that's stuck. TODO(verify) on the floor at the approach speed.
const unsigned long APPROACH_PROGRESS_MS = 1000;
const int APPROACH_MIN_CLOSE_MM          = 30;
const unsigned long APPROACH_GIVEUP_SUPPRESS_MS = 3000;

// SCAN only starts with this much clear space on BOTH halves of the 8x8
// (28/9 it started with a wall 30cm away and spun toward it)
const int SCAN_CLEAR_MM = 600;

// ---------------------------------------------------------------------------
// PICKUP TRIGGER (inductive) / CREEP / REJECT  -  see navigation.cpp
// Notch bench test 2026-09-25: bottom-left saw an upright steel weight at
// ~139mm, but neither bottom ToF sees it once it's IN the notch, so the
// robot creeps through that blind gap and the inductive sensor (only reads
// metal within ~7mm) triggers the pickup.
// TODO(verify) on the floor: creep speed/time, reverse distance.
// ---------------------------------------------------------------------------
const unsigned long INDUCTIVE_CONFIRM_MS = 60;    // metal must read steadily this long
const int  CREEP_START_MM   = 200;   // candidate lost closer than this -> creep, not give up
const int  CREEP_SPEED_PCT  = 30;
const unsigned long CREEP_MAX_MS = 1000;   // no metal by then -> REJECT
const int  MAX_PICKUP_TRIES = 3;
// after giving up on a weight, the inductive trigger stays locked until the
// notch has read clear this long (else a stuck weight is retried forever)
const unsigned long METAL_REARM_CLEAR_MS = 500;     // metal still in the notch after a cycle = missed grab -> retry
const int  REJECT_REVERSE_PCT = 40;
const unsigned long REJECT_REVERSE_MS  = 500;    // stack_test / notchtest only - nav reverses by distance (below)
const unsigned long REJECT_SUPPRESS_MS = 4000;   // ignore that spot for this long
// nav REJECT (2026-09-29): the timed 500ms reverse left the robot trapped on
// the rejected object. Now: reverse REJECT_REVERSE_MM by the encoders (raw,
// not slip-corrected), then pivot REJECT_PIVOT_DEG by the IMU. The pivot is
// cut short to approach a DIFFERENT weight the bottom pair spots, but only
// once it has turned REJECT_NEW_WEIGHT_MIN_DEG - before that, a candidate is
// most likely the rejected object itself. Live-tune with ':rev' / ':pdeg'.
// TODO(verify) all on the arena floor.
const int REJECT_REVERSE_MM                = 200;
const unsigned long REJECT_REVERSE_MAX_MS  = 2500;  // safety cap (tracks stalled/slipping)
const int REJECT_PIVOT_DEG                 = 90;
const int REJECT_PIVOT_LEAD_DEG            = 8;     // stop this early - the soft stop carries it on
const unsigned long REJECT_PIVOT_MAX_MS    = 3000;  // safety cap; without the IMU it's timed (REPOSITION_TURN_MS)
const int REJECT_NEW_WEIGHT_MIN_DEG        = 35;

// ---------------------------------------------------------------------------
// LYING-WEIGHT REJECT  -  top/bottom baseplate ToF pair (TOF_TOP, proposed,
// see above) + the existing notch ToF (TOF_UPRIGHT) as "bottom". TOF_TOP is
// bench-confirmed blind to a weight lying on its side; TOF_UPRIGHT still
// sees it. So bottom-sees-something + top-sees-nothing, held steadily, means
// "an object is here but not standing up" - reject it before creeping it
// through the funnel, rather than waiting to find out at the inductive
// sensor. See navigation.cpp lyingWeightConfirmed().
//
// This replaces the old inductive-gated, settle-timer-based back-away check
// (uprightBackawayConfirmed(), now removed from navigation.cpp - the
// UPRIGHT_BACKAWAY_* constants above are only used by the deferred notchtest
// bench rig, BENCH_TODO.md 2e). The second sensor makes that settle-window
// trick unnecessary: orientation now comes from an independent reading
// instead of being inferred from how a distance changed over time, which is
// what was causing real upright weights to get rejected too quickly.
//
// 2026-09-29: rewritten to match the patient, discrepancy-based logic
// proven out in stack_test.cpp (BENCH_TODO.md 2h) rather than the original
// simple "both within a fixed band" check. Mismatch = bottom present with
// top NEVER confirming (the real lying-weight signature - top only ever
// sees an UPRIGHT weight's top edge), OR both valid but far apart
// (LYING_DISCREPANCY_MM, a weaker secondary signal). Either resets the
// instant top gets ANY valid reading - that's active evidence the weight
// is still arriving, not a fixed timeout that can't tell "still arriving"
// from "lying down". LYING_CONFIRM_MS is deliberately long (patience over
// speed) - see navigation.cpp lyingWeightConfirmed().
//
// 2026-09-29 (integration): nav now runs the check EXACTLY as stack_test.cpp
// does - these constants are the single source for both (the STACK_* ones at
// the end of this file are aliases of them):
//   - "present" = 0 < mm < ceiling for BOTH sensors. 0 is "nothing in range",
//     never "present"; the ceiling rejects far-field returns (both are long-
//     range L1X - on open floor the notch locked onto the far wall and fired
//     REJECT while just driving forward, 29/9 arena).
//   - the mismatch timer runs every loop, not only while it's being asked.
//   - it is only acted on in CREEP, which starts once the notch ToF sees
//     something (or the bottom pair loses a close candidate, as before).
//   - once the notch has seen something, CREEP gets CREEP_HARD_CAP_MS (not
//     CREEP_MAX_MS) - the patience window has to fit inside it.
// TODO(verify): starting points from stack_test.cpp's bench/arena testing -
// live-tune with nav_main's ':disc'/':lconf'/':ccap'/':rev'/':turn'.
const int LYING_NOTCH_MAX_MM   = 150;  // notch (TOF_UPRIGHT) presence ceiling. Upright 59, dummy 70-74, lying 110-149 (28/9)
const int LYING_TOP_MAX_MM     = 150;  // top (TOF_TOP) presence ceiling
const int LYING_DISCREPANCY_MM = 40;   // |notch_mm - top_mm| above this = mismatch (secondary signal)
const unsigned long LYING_CONFIRM_MS  = 1500;  // was 100 - too short, rejected real weights still arriving
const unsigned long CREEP_HARD_CAP_MS = 8000;  // CREEP give-up once the notch has seen something
const unsigned long NOTCH_IGNORE_MS   = REJECT_SUPPRESS_MS;  // after a REJECT, the notch can't restart CREEP for this long

// ---------------------------------------------------------------------------
// PD STEERING for APPROACH (mm imbalance -> % differential)
// ---------------------------------------------------------------------------
const float KP_APPROACH = 0.16f;
const float KD_APPROACH = 0.045f;
const int   APPROACH_STEER_MAX = 22;
const int   ONE_SIDE_ARC_PCT   = 16;

// ---------------------------------------------------------------------------
// PICKUP / REPOSITION / suppression
// ---------------------------------------------------------------------------
// after a pickup ATTEMPT (regardless of confirmed success - a dummy weight
// won't be grabbed by the magnets, and we have no way to tell at ground
// level), suppress re-detecting the same spot so we don't loop on it.
const unsigned long TARGET_SUPPRESS_MS = 1500;

// ---------------------------------------------------------------------------
// CRANE (collection.cpp)  -  angles in degrees, speeds in degrees/second.
// Tuned by the partner on the robot 2026-09-24:
//   pickup 120 @ 45 deg/s - slower avoids the arm swinging inconsistently
//                           as it comes down onto the weight
//   drop    40 @ 60 deg/s - middle ground carrying the weight to storage;
//                           40 drops it cleanly onto the ramp
// All six can be changed live on the bench: pio run -e servotest (see
// servo_test.cpp), then paste the printed values back here.
// ---------------------------------------------------------------------------
const int   CRANE_PICKUP_ANGLE = 113;   // was 118 (2026-09-29: reduced 5 deg to increase magnet contact)
const unsigned long CRANE_PICKUP_SEAT_MS = 1500; // full wait at pickup angle before lifting
const float CRANE_PICKUP_DPS   = 45.0f;
const int   CRANE_DROP_ANGLE   = 40;
const float CRANE_DROP_DPS     = 60.0f;
const int   CRANE_REST_ANGLE   = 70;     // rest angle + speed cotrnfirmed by the partner 2026-09-25
const float CRANE_REST_DPS     = 100.0f;

// Crane pins (used by collection.cpp).
const int PIN_CRANE_SERVO = 15;   // TODO(verify): moved from pin 28 (CON67) - confirm which connector pin 15 is on the board
// Single electromagnet via the FET board (reverted from the two-magnet array
// - team decision, 2026-09-28; CLAUDE.md corrected to match). Moved from
// CON74 (pin 26, no PWM on the Teensy 4.0) to CON72 (pin 24, PWM-capable) so
// a reduced holding level can be used for the 3rd carried target - see
// BENCH_TODO.md 2d. Pin 24 doubles as Wire2's SCL2, unused elsewhere.
const int PIN_MAGNET = 24;        // CON72
// Confirmed by bench shake test (magnettest env, 2026-09-28): 50% duty held
// through a shake. Used to carry the 3rd target on the arm at rest instead of
// dropping it into storage - see COLLECTION_HOLD in collection.cpp.
const int MAGNET_HOLD_PCT = 60;   // % duty while carrying the 3rd target

// eased servo moves (smooth_servo.cpp) - used by the crane
const unsigned long SERVO_MIN_MOVE_MS     = 150;  // floor for tiny moves
const unsigned long SERVO_STEP_INTERVAL_MS = 15;  // angle update period during a move

// A full crane cycle is about 5s including the 1.5s seated wait; give up
// waiting after this so a stuck crane can't park the robot for the round
const unsigned long PICKUP_TIMEOUT_MS = 8000;

// ---------------------------------------------------------------------------
// ROUND  -  start trigger, 2-minute timer, on-board target cap
// ---------------------------------------------------------------------------
// GO button (blue) on CON68 = A0Z = pin 14. Measured 2026-09-25 with the
// nav build's goD/goA telemetry: released ~0.6V (digital 0), pressed ~3.0V
// (digital 1) -> active HIGH. The button board drives both levels, so no
// pull-up is used. Set PIN_GO to -1 to fall back to auto-starting
// AUTO_START_DELAY_MS after boot.
const int  PIN_GO           = 14;
const bool GO_ACTIVE_LOW    = false;
const unsigned long GO_DEBOUNCE_MS      = 50;
// BENCH/FLOOR TESTING ONLY: a 2nd GO press during a round ends it (motors
// soft-stop, round OVER). Lets you stop an untethered test without USB.
// TODO(competition): set false - no human intervention is allowed, and a
// stray press must never end a real round.
const bool GO_STOPS_ROUND = true;
const unsigned long AUTO_START_DELAY_MS = 3000;

const unsigned long ROUND_MS        = 120000;
const unsigned long ROUND_END_MARGIN_MS = 1500;  // stop this early - finish still
// late-round: stop collecting and head home. Only applies once homing
// exists (USE_HOMING 1) - without it, stopping early just wastes pickups,
// since anything on board still scores 1x.
#define USE_HOMING 0
const unsigned long RETURN_HOME_AT_MS = 95000;
// rules: >3 targets on board = -1 each. Stop collecting at the cap.
const int MAX_TARGETS_ON_BOARD = 3;

// ---------------------------------------------------------------------------
// SEARCH SCAN  (timed - no IMU on this rig yet, see "IMU HOOK" comments)
// ---------------------------------------------------------------------------
const unsigned long SCAN_TRIGGER_MS = 6000;
const unsigned long SCAN_MAX_MS     = 2600;

// ---------------------------------------------------------------------------
// SCHEDULING (ms)
// ---------------------------------------------------------------------------
const unsigned long TOF_READ_MS  = 55;
const unsigned long IR_READ_MS   = 30;
const unsigned long TELEMETRY_MS = 200;

// ---------------------------------------------------------------------------
// STACK TEST (stack_test.cpp, [env:stacktest]) - SEARCH (drive + steer toward
// a weight seen by the two outer weight-search ToFs) -> CREEP (once the
// underside notch ToF sees something) -> confirm against the top ToF ->
// REVERSE (rejected) or keep going until the inductive sensor fires, which
// overrides everything and starts collection immediately. Bench rig only -
// does NOT touch tof.cpp/navigation.cpp. The four ToFs here are the same
// physical sensors as the main build's TOF_BL/TOF_BR/TOF_UPRIGHT/TOF_TOP -
// only one env is ever flashed at a time, so no runtime conflict.
//
// Geometry (from the team's sensor layout drawing, 2026-09-28): top plate at
// 34mm, all ToFs at 57mm up EXCEPT the underside/notch ToF (13mm) and the
// inductive sensor (50mm). The 13mm notch beam crosses both an upright
// weight's lower body and a lying weight's silhouette (max ~50mm tall on its
// side) - it can't tell them apart alone. The 57mm top ToF only intersects
// an upright weight's TOP 13mm (57-70mm) - never a lying one - but that also
// means it legitimately reads NOTHING for most of a real upright weight's
// creep-in, until it's nearly fully seated. That's why this can't be a short
// fixed timeout: "notch present, top absent" is the NORMAL state while a
// real weight is still arriving, not just the lying-weight signature. The
// distinguishing test is whether it EVER resolves as the creep continues.
// ---------------------------------------------------------------------------
const int STACK_XSHUT_NOTCH    = 0;   // CON27, L1X, underside ~13 mm up, cone up
const int STACK_XSHUT_TOP      = 4;   // CON31, L1X, 57 mm up, cone down
const int STACK_XSHUT_SEARCH_L = 2;   // CON29, L0X, outer weight-search, left
const int STACK_XSHUT_SEARCH_R = 1;   // CON28, L0X, outer weight-search, right
const int STACK_TOP_ROI        = 8;   // L1X ROI side in SPADs (4..16, 16 = full). TODO(verify): starting guess

// SEARCH: drive forward, arc toward whichever outer ToF sees something
// closer within this range. TODO(verify): starting guesses, not bench-tuned.
const int STACK_SEARCH_MAX_MM    = 300;
const int STACK_SEARCH_SPEED_PCT = 40;
const int STACK_SEARCH_ARC_PCT   = 16;   // matches ONE_SIDE_ARC_PCT's magnitude

// Presence ceilings for notch/top (2026-09-29 arena test finding): both are
// long-range-capable L1X sensors, so "mm > 0" alone isn't "something's at
// the notch" - on open floor notch was locking onto the far wall/bare floor
// at real, valid, non-zero distances, reading as a permanent false
// "present", which then mismatched against top (correctly seeing nothing)
// and fired REVERSE while just driving forward. A ceiling rejects any
// return beyond plausible notch/top range as "nothing", same as the earlier
// funnel/BOTTOM_PRESENT_MM-style thresholds elsewhere in this project.
// TODO(verify): starting point from the bench data in the geometry comment
// above (matched cases read 60-110mm, lying up to ~130mm) - retune on the
// arena floor with 'h'/'i'.
// Same values as nav - aliases of the LYING_* constants (one source of truth).
const int STACK_NOTCH_MAX_MM = LYING_NOTCH_MAX_MM;
const int STACK_TOP_MAX_MM   = LYING_TOP_MAX_MM;

// Confirm logic: "mismatch" = notch present with top NEVER confirming (the
// expected lying-weight signature), OR both valid but far apart (a weaker
// secondary signal - e.g. a partial/angled top return). Either resets to
// zero the instant top produces ANY valid reading, since that's active
// evidence the weight is still arriving, not lying down.
// TODO(verify): both values are starting guesses - tune live with 'e'/'f'
// while re-presenting empty/real/dummy/lying, watching the 'diff'/'mismatch'
// telemetry columns, THEN paste the values back here. STACK_MISMATCH_CONFIRM_MS
// is deliberately long (patience over speed, per team decision 2026-09-28) -
// it only needs to be shorter than STACK_CREEP_MAX_MS, the hard fallback cap.
const int STACK_DISCREPANCY_MM = LYING_DISCREPANCY_MM;          // |notch_mm - top_mm| above this = mismatch (secondary signal)
const unsigned long STACK_MISMATCH_CONFIRM_MS = LYING_CONFIRM_MS;
const unsigned long STACK_CREEP_MAX_MS        = CREEP_HARD_CAP_MS;  // hard cap regardless of the above - safety net, not the normal path
