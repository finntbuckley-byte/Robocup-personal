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
//      4x ToF via SX1509 XSHUT lines (ToF connectors CON27-34 = XSHUT0-7):
//        bottom-left  VL53L0X  CON29  - weight detection / APPROACH steering
//        bottom-right VL53L0X  CON28  - weight detection / APPROACH steering
//        weight-detect (upright) CON27 - upright vs lying weight in the notch
//        rear         VL53L1X  CON30  - reversing clearance
//      1x SEN0628 8x8 ToF, front, CON64 (RAW I2C1 = Wire1) - obstacles
//      2x analog IR (GP2Y0A21, white), side-facing: left CON24 (A9Z),
//        right CON23 (A8Z)
//      1x inductive proximity sensor at the end of the funnel, via the
//        inductive level-shift board to CON70 (A6Z, pin 20) - metal
//        (real weight) vs non-metal (dummy) discrimination
//    Not fitted: IMU (left out for now), TCS34725 colour sensor (dropped).
//    The old top-front + corner ToFs are gone - the SEN0628 does their job.
//
//  BUILDS: [env:nav] = full navigation (nav_main.cpp). The default env is
//  still the partner's collection test (main.cpp).
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
//                all 4 ToF XSHUT lines, SEN0628 port, side IR pins + L/R +
//                part, inductive pin, gate on CON67 (connector map,
//                2026-09-24).
//    TODO(verify) ToF model on the weight-detect port; which RAW bus the ToF
//                bus (CON35) and XSHUT expander (CON26) are cabled to - the
//                wirefind report answers both.
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// MODULE ENABLE SWITCHES
// ---------------------------------------------------------------------------
#define USE_REAR_TOF    1
#define USE_SIDE_IR     1
#define USE_FUNNEL_SORT 1     // funnel ToF + inductive classification
#define USE_SCAN        1

// ---------------------------------------------------------------------------
// ROBOT GEOMETRY  (mm)
// ---------------------------------------------------------------------------
const int ROBOT_WIDTH_MM   = 165;
const int ROBOT_HEIGHT_MM  = 170;
const int WEIGHT_HEIGHT_MM = 70;

// ---------------------------------------------------------------------------
// ToF CHAIN  -  4 sensors, XSHUT via SX1509 @ 0x3F
// ---------------------------------------------------------------------------
const byte SX1509_ADDRESS = 0x3F;
// re-addressed from 0x34 upward in index order - 0x30 would put index 3 on
// 0x33, the SEN0628's fixed address
#define TOF_ADDRESS_START 0x34

// index order used EVERYWHERE:
const int TOF_BL      = 0;   // bottom-left  VL53L0X          CON29 / XSHUT2
const int TOF_BR      = 1;   // bottom-right VL53L0X          CON28 / XSHUT1
const int TOF_UPRIGHT = 2;   // weight-detect, across notch   CON27 / XSHUT0
const int TOF_REAR    = 3;   // rear VL53L1X, long range      CON30 / XSHUT3
const int TOF_COUNT   = 4;

// SX1509 XSHUT line for each index (connector CONn = XSHUT(n-27))
const int XSHUT_TOF[TOF_COUNT] = { 2, 1, 0, 3 };

// sensor model at each index: 0 = VL53L0X (short), 1 = VL53L1X (long)
// TODO(verify): weight-detect model - nav_test assumed a VL53L1X in short
// mode; the wirefind report prints the real part per XSHUT line.
const int TOF_TYPE[TOF_COUNT] = { 0, 0, 1, 1 };

// VL53L1X distance mode per index (ignored for L0X): true = Short (to ~1.3m,
// better in ambient light), false = Long (to ~4m). The notch sensor only
// looks ~60-90mm, the rear wants range.
const bool TOF_SHORT_MODE[TOF_COUNT] = { false, false, true, false };

// Which RAW I2C bus each sub-assembly is cabled to. Only the ToFs (via I2C In
// CON35) and the XSHUT expander (via I2C In CON26) matter here.
// TODO(verify): run wirefind - its report names the bus for both. RAW I2C0 =
// Wire, RAW I2C1 = Wire1.
#define TOF_WIRE Wire
#define SX_WIRE  Wire

// SX1509 registers (bank A = I/O 0-7 = XSHUT0-7)
const byte SX_REG_DIR_A  = 0x0F;
const byte SX_REG_DATA_A = 0x11;

const unsigned long TOF_BOOT_MS      = 50;   // after raising XSHUT - proven necessary
const unsigned long TOF_PERIOD_MS    = 50;   // continuous-mode inter-measurement period
const unsigned long TOF_L1X_BUDGET_US = 33000;
const uint16_t TOF_MAX_VALID_MM = 4000;   // VL53L1X long-mode max; above = garbage

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
// the grid matches reality. TODO(verify) all four on the robot.
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

// ---------------------------------------------------------------------------
// INDUCTIVE SENSOR  -  metal (real weight) vs non-metal (dummy) at the
// funnel end. LJ18A3-8-Z/BY -> inductive interface/level-shift board ->
// CON70 (A6Z) = pin 20, read as digital. Active LOW matches
// Inductive_sensor.cpp's tested METALLIC = 0.
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
// 1050us reverse. motor.cpp maps 100% to 1.0/2.0 ms, so drive() SCALES its
// -100..100 command into these limits (keeps steering differential at full
// cruise). From motor.cpp's mapping (integer maths, so 1us inside the limit):
//   fwd: 1842 + 68*158/100 = 1949 us     rev: 1188 - 73*188/100 = 1051 us
const int MOTOR_MAX_FWD_PCT = 68;
const int MOTOR_MAX_REV_PCT = 73;

// Scales EVERY drive command (after navigation, before the pulse caps).
// 50 for first floor tests; set back to 100 once avoidance behaves.
const int DRIVE_SCALE_PCT = 50;

// ---------------------------------------------------------------------------
// WEIGHT DETECTION  (bottom ToF vs 8x8 obstacle band, same side)
// ---------------------------------------------------------------------------
// A bottom ToF return with nothing at a similar distance in the 8x8's
// obstacle band on that side = something short = weight candidate.
// KNOWN LIMITATION: a weight within DIFF_CLEAR_MARGIN_MM of a wall reads as
// wall, so weights against walls are ignored.
const int WEIGHT_MIN_MM        = 60;
const int WEIGHT_MAX_MM        = 700;
const int DIFF_CLEAR_MARGIN_MM = 250;
const int WEIGHT_STICK_MS      = 120;

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
const int  MAX_PICKUP_TRIES = 2;     // metal still in the notch after a cycle = missed grab -> retry
const int  REJECT_REVERSE_PCT = 40;
const unsigned long REJECT_REVERSE_MS  = 500;    // aim ~10cm clear of the notch
const unsigned long REJECT_SUPPRESS_MS = 4000;   // ignore that spot for this long

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
const int   CRANE_PICKUP_ANGLE = 118;   // confirmed with a weight 2026-09-25 (less stall than 120, which is the fallback)
const float CRANE_PICKUP_DPS   = 45.0f;
const int   CRANE_DROP_ANGLE   = 40;
const float CRANE_DROP_DPS     = 60.0f;
const int   CRANE_REST_ANGLE   = 70;     // rest angle + speed confirmed by the partner 2026-09-25
const float CRANE_REST_DPS     = 100.0f;

// eased servo moves (smooth_servo.cpp) - used by the crane
const unsigned long SERVO_MIN_MOVE_MS     = 150;  // floor for tiny moves
const unsigned long SERVO_STEP_INTERVAL_MS = 15;  // angle update period during a move

// a full crane cycle is ~4.6s (SERVODELAY1+2 + DROP + SERVODELAY3); give up
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
