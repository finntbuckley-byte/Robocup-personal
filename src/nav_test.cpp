/* ============================================================================
 *  nav_test.cpp  -  RoboCup Group 23 drive/navigation bring-up rig
 *
 *  Build:  pio run -e navtest -t upload
 *  Serial: 115200, press '?' for the menu.
 *
 *  Compiles ONLY this file + motor.cpp, so it can't clash with main.cpp /
 *  imu.cpp / simple_main.cpp (all three define setup()/loop()).
 *
 *  ===========================================================================
 *  SENSOR LAYOUT - this build follows the current plate drawings, NOT the
 *  older 7-ToF arrangement in config.h.
 *  ===========================================================================
 *
 *  BOTTOM PLATE (front = the V-notch end)
 *    2x VL53L0X  short range, front corners, flanking the notch mouth,
 *                angled outward. Confirms a weight is entering the funnel
 *                and which side it is biased toward.       XSHUT 4 and 3
 *    1x VL53L1X  short range + 4x4 ROI, across the notch, 30mm above the
 *                plate top, tilted up 8 degrees. Upright (70mm) vs lying
 *                (50mm) check. Replaces the HC-SR04.              XSHUT 5
 *    1x inductive proximity at the notch apex - steel vs plastic dummy.
 *    2x analog IR, one per side edge, facing out - wall following.
 *                OR 2x VL53L1X per side if USE_SIDE_TOF is 1 - see below.
 *
 *  TOP PLATE
 *    1x DFRobot SEN0628 8x8 matrix ToF, FRONT facing, I2C 0x33 - weight and
 *       obstacle scanning, bearing to target, wall vs weight discrimination.
 *    1x VL53L1X  long range, REAR facing - reversing clearance.    XSHUT 6
 *    1x BNO055 IMU, centre of plate, on Wire1 - heading, pitch.
 *
 *  ===========================================================================
 *  BOARD MAP  (Parts Summary 2026B, pp.26-32)
 *  ===========================================================================
 *  Teensy 4.0, 3.3V only, NOT 5V tolerant.
 *
 *  DC motor driver (203_DCMotor): servo-style PPM. motor.cpp uses D0 (left)
 *      and D1 (right) = SERIAL1 port, CON65.
 *  ToF XSHUT: SX1509 @0x3F, TOF ports CON27-CON34 = XSHUT0..XSHUT7.
 *  8x8 array: I2C0, fixed 0x33, needs DFRobot_MatrixLidar.
 *  IMU: Wire1 (I2C 1 ports CON62/63/64) @0x28, so it never shares the ToF bus.
 *  Side IR: analogue ports A8Z = CON23, A9Z = CON24.
 *  Inductive: through 503_Inductive_Interface (needs 11V from the power
 *      module, channels A/B for the large green sensor) into pin 20 = A6Z,
 *      CON70. Pin 20 is what your Working_inductive_sensor_test.ino uses.
 *
 *  ===========================================================================
 *  THREE PROBLEMS IN THE EXISTING CODE - read before flashing
 *  ===========================================================================
 *  1. ADDRESS COLLISION. tof.cpp re-addresses the ToFs from 0x30 upward.
 *     The 8x8 array is fixed at 0x33 (Parts Summary p.28), so a fifth ToF
 *     would land on top of it. This build starts the ToF block at 0x34
 *     (still inside the 0x30-0x38 reassign space) to stay clear.
 *
 *  2. IR CURVE 100x OUT. config.h has IR_SIDE_A = 12080 with the result
 *     used as mm. The GP2Y0A41SK fit is distance_cm = 12.08 * V^-1.058, so
 *     in mm the coefficient is 120.8. As written every reading falls outside
 *     the 40-300mm clamp and returns 0 = "clear", so the side nudge has
 *     never fired. This build uses the 2Y0A02 curve (the part in the
 *     drawing) with the coefficient in mm.
 *
 *  3. MOTOR PULSE LIMITS. Parts Summary p.42 gives 1.05ms full reverse,
 *     1.5ms stop, 1.95ms full forward, and says the controller ignores
 *     anything outside its valid range. motor.cpp uses 1000 and 2000. Test
 *     '2' ramps through the range so you can see whether 100% really is
 *     full speed. If the top misbehaves, change FULL_FORWARD to 1950 and
 *     FULL_BACKWARD to 1050 in motor.cpp.
 *
 *  Anything marked CONFIRM was not verifiable from the repo.
 * ==========================================================================*/













/* <--------------- REMOVE THIS TO UNCOMMENT THE HOLE THING         












#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "DFRobot_MatrixLidar.h"
#include "motor.h"          // partner's code, unchanged

// ============================================================================
//  CONFIG
// ============================================================================

static const uint8_t SX1509_TOF_ADDR = 0x3F;   // XSHUT0-7 + BIO8-12
static const uint8_t TOF_ADDR_BASE   = 0x34;   // 0x34 up - clears the 8x8 at 0x33
                                               // (0x34..0x39 with side ToFs on;
                                               //  nothing else in the kit sits there)
static const uint8_t X8_ADDR         = 0x33;   // SEN0628, fixed
static const uint8_t BNO055_ADDR     = 0x28;   // on Wire1

// ---- Side sensing: IR or ToF -------------------------------------------
// 0 = analog IR on the sides (default, matches the drawing)
// 1 = VL53L1X per side instead. Better for wall following - narrow beam,
//     linear, no reflectivity guesswork, no 100mm blind zone - but it needs
//     two spare L1X modules and two free XSHUT channels. The IR pins are
//     still read and printed either way, so you can compare them directly.
#define USE_SIDE_TOF 0

// ---- ToF array ----------------------------------------------------------
// XSHUT 4 and 3 are CONFIRMED from your tof.cpp. The rest are free channels
// picked for the new positions - CONFIRM against your wiring.
enum {
  T_FUNL = 0, T_FUNR, T_UPRIGHT, T_REAR,
#if USE_SIDE_TOF
  T_SIDE_L, T_SIDE_R,
#endif
  T_COUNT
};

struct TofCfg { const char *name; int8_t xshut; bool isL1X; bool shortMode; };

static const TofCfg TOF_CFG[T_COUNT] = {
  { "FUN_L",   4, false, false },  // bottom front-left  VL53L0X, angled out
  { "FUN_R",   3, false, false },  // bottom front-right VL53L0X, angled out
  { "UPRIGHT", 5, true,  true  },  // across the notch, short mode + 4x4 ROI
  { "REAR",    6, true,  false },  // top plate, rear facing, long mode
#if USE_SIDE_TOF
  { "SIDE_L",  2, true,  false },  // left side, long mode  - CONFIRM
  { "SIDE_R",  1, true,  false },  // right side, long mode - CONFIRM
#endif
};

// ---- Upright check ------------------------------------------------------
// Lens 30mm above the plate top (57mm above ground), tilted up 8 degrees,
// weight face 60-70mm away. Upright weights break the beam, lying ones pass
// under it. Threshold sits well short of the empty-channel background.
static const uint16_t UPRIGHT_TRIGGER_MM = 90;
static const uint16_t UPRIGHT_MIN_MM     = 40;   // L1X short-mode floor

// ---- 8x8 matrix ToF -----------------------------------------------------
#define USE_TOF_X8 1
static const float X8_FOV_DEG = 63.0f;           // CONFIRM against datasheet
static const float X8_ZONE_DEG = X8_FOV_DEG / 8.0f;
// Zone (row, col) orientation depends on how the module is mounted. Run
// test 'a', wave your hand top-left of the sensor, and see which corner
// lights up. Flip these until the grid matches reality.
#define X8_COL_SIGN  (+1.0f)    // +1: column 7 is to the robot's right
#define X8_ROW_FLIP  0          // 1: row 0 is the BOTTOM of the field
static const uint8_t X8_BAND_LO = 2;   // rows used for obstacle/weight work
static const uint8_t X8_BAND_HI = 5;
static const uint16_t X8_MAX_VALID_MM = 3000;
static const uint16_t X8_WALL_SPREAD_MM = 150;   // flat across columns = wall

// ---- Analog IR ----------------------------------------------------------
// Pick the part actually fitted. 2Y0A21 (white, 100-800mm) is the best fit
// for this arena: wall following runs at 150-400mm, which sits in the middle
// of its curve, and its blind zone is 100mm instead of the 2Y0A02's 200mm.
// All three fits are mm = A * V^B, i.e. the published cm fit x10.
//   1 = 2Y0A21   white   100-800mm   (recommended)
//   2 = 2Y0A02   blue    200-1500mm
//   3 = 0A41SK   yellow   40-300mm
#define IR_PART 1

#if   IR_PART == 1
  static const float IR_A = 277.28f;   // cm fit 27.728 * V^-1.2045
  static const float IR_B = -1.2045f;
  static const int   IR_MIN_MM = 100;
  static const int   IR_MAX_MM = 800;
#elif IR_PART == 2
  static const float IR_A = 603.74f;   // cm fit 60.374 * V^-1.16
  static const float IR_B = -1.16f;
  static const int   IR_MIN_MM = 200;
  static const int   IR_MAX_MM = 1500;
#else
  static const float IR_A = 120.8f;    // cm fit 12.08 * V^-1.058  (NOT 12080)
  static const float IR_B = -1.058f;
  static const int   IR_MIN_MM = 40;
  static const int   IR_MAX_MM = 300;
#endif

static const int PIN_IR_L = A8;        // CON23 (A8Z) - CONFIRM which side
static const int PIN_IR_R = A9;        // CON24 (A9Z) - CONFIRM which side
static const int   ADC_BITS = 12;
static const float ADC_VREF = 3.3f;
static const float ADC_MAX  = 4095.0f;
static const int   IR_SAMPLES = 5;

// ---- Inductive ----------------------------------------------------------
static const int  PIN_INDUCTIVE = 20;           // A6Z, CON70 - from your test
static const bool INDUCTIVE_ACTIVE_LOW = true;  // NPN sensors pull low on metal

// ---- Drive tuning -------------------------------------------------------
static const int CRUISE_PCT   = 45;    // test speed, deliberately not 100
static const int APPROACH_PCT = 30;
static const int TURN_PCT     = 40;
static const int MIN_TURN_PCT = 22;    // below this the tracks just buzz

// ---- Heading control ----------------------------------------------------
static const float HEADING_SIGN = +1.0f;   // -1 if a right turn drops the yaw
static const float KP_HEADING   = 1.6f;    // %/deg
static const int   HEAD_CORR_MAX = 30;
static const float TURN_TOL_DEG  = 3.0f;
static const unsigned long TURN_SETTLE_MS = 250;

// ---- Wall following (side IR) -------------------------------------------
static const int   WALL_TARGET_MM = 300;
static const float KP_WALL = 0.10f;        // %/mm
static const int   WALL_CORR_MAX = 25;

// ---- Approach steering (8x8 bearing) ------------------------------------
static const float KP_BEARING = 1.3f;      // %/deg
static const int   BEARING_CORR_MAX = 25;

// ---- Stopping distances -------------------------------------------------
static const int FRONT_STOP_MM   = 250;
static const int REAR_STOP_MM    = 250;
static const int PICKUP_TRIGGER_MM = 115;  // bottom pair says it's in the notch

// ---- Housekeeping -------------------------------------------------------
static const unsigned long TEST_TIMEOUT_MS = 10000;
static const unsigned long TELEMETRY_MS    = 200;
static const unsigned long TOF_READ_MS     = 55;
static const unsigned long IR_READ_MS      = 30;
static const unsigned long X8_READ_MS      = 100;

// ============================================================================
//  GLOBALS
// ============================================================================

SX1509 io;
static VL53L0X l0x[T_COUNT];
static VL53L1X l1x[T_COUNT];
static uint16_t tofMM[T_COUNT] = { 0 };
static bool tofOk[T_COUNT] = { false };

#if USE_TOF_X8
DFRobot_MatrixLidar_I2C x8(X8_ADDR, &Wire);
static uint16_t x8Grid[64] = { 0 };
static bool x8Ok = false;
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire1);
static bool  imuOk = false;
static float headingDeg = 0.0f, pitchDeg = 0.0f;
static uint8_t calSys = 0, calGyro = 0, calAcc = 0, calMag = 0;

static uint16_t irLmm = 0, irRmm = 0;
static bool inductiveMetal = false;

static int lastL = 0, lastR = 0;

// ============================================================================
//  DRIVE
// ============================================================================

static inline int clampPct(int v) { return v > 100 ? 100 : (v < -100 ? -100 : v); }

// signed percent per side -> motor.cpp's magnitude + direction calls
static void drive(int leftPct, int rightPct)
{
  lastL = clampPct(leftPct);
  lastR = clampPct(rightPct);

  if (lastL > 0)      motorForward(lastL, 1);
  else if (lastL < 0) motorBackward(-lastL, 1);
  else                motorStop(1);

  if (lastR > 0)      motorForward(lastR, 2);
  else if (lastR < 0) motorBackward(-lastR, 2);
  else                motorStop(2);
}

static void stopAll() { drive(0, 0); }

// ============================================================================
//  ToF (XSHUT chain)
// ============================================================================

// The Pololu VL53L1X library exposes no ROI API, so the 4x4 window for the
// upright check is written straight to the ST registers. 0x0080 packs
// (y-1)<<4 | (x-1); 0x007F is the centre SPAD (199 = array centre).
static void setRoi4x4(VL53L1X &s)
{
  s.writeReg(0x007F, 199);
  s.writeReg(0x0080, (uint8_t)(((4 - 1) << 4) | (4 - 1)));
}

static void tofInitAll()
{
  if (!io.begin(SX1509_TOF_ADDR)) {
    Serial.println("!! SX1509 @0x3F not found - check its 4-pin I2C cable");
    return;
  }

  // hold everything in reset first, or they all answer on 0x29 at once
  for (int i = 0; i < T_COUNT; i++) {
    if (TOF_CFG[i].xshut < 0) continue;
    io.pinMode(TOF_CFG[i].xshut, OUTPUT);
    io.digitalWrite(TOF_CFG[i].xshut, LOW);
  }
  delay(10);

  for (int i = 0; i < T_COUNT; i++) {
    if (TOF_CFG[i].xshut < 0) continue;

    io.digitalWrite(TOF_CFG[i].xshut, HIGH);
    delay(50);                                   // boot time, don't shorten

    bool ok = false;
    if (!TOF_CFG[i].isL1X) {
      l0x[i].setTimeout(100);
      ok = l0x[i].init();
      if (ok) {
        l0x[i].setAddress(TOF_ADDR_BASE + i);
        l0x[i].startContinuous(50);
      }
    } else {
      l1x[i].setTimeout(100);
      ok = l1x[i].init();
      if (ok) {
        l1x[i].setAddress(TOF_ADDR_BASE + i);
        if (TOF_CFG[i].shortMode) {
          l1x[i].setDistanceMode(VL53L1X::Short);
          l1x[i].setMeasurementTimingBudget(20000);
          setRoi4x4(l1x[i]);                     // ~15 deg cone
        } else {
          l1x[i].setDistanceMode(VL53L1X::Long);
          l1x[i].setMeasurementTimingBudget(50000);
        }
        l1x[i].startContinuous(50);
      }
    }

    tofOk[i] = ok;
    Serial.printf("ToF %-8s XSHUT%d %s @0x%02X  %s\n", TOF_CFG[i].name,
                  TOF_CFG[i].xshut, TOF_CFG[i].isL1X ? "L1X" : "L0X",
                  TOF_ADDR_BASE + i, ok ? "ok" : "FAILED");
  }
}

static void tofUpdate()
{
  static unsigned long last = 0;

  for (int i = 0; i < T_COUNT; i++)
    if (tofOk[i] && TOF_CFG[i].isL1X && l1x[i].dataReady())
      tofMM[i] = l1x[i].read(false);

  if (millis() - last < TOF_READ_MS) return;
  last = millis();

  for (int i = 0; i < T_COUNT; i++)
    if (tofOk[i] && !TOF_CFG[i].isL1X) {
      uint16_t mm = l0x[i].readRangeContinuousMillimeters();
      tofMM[i] = (mm >= 8000) ? 0 : mm;          // 8190 = no target
    }
}

// 0 means "nothing in range", so it must not win a min() comparison
static uint16_t nearest(uint16_t a, uint16_t b)
{
  if (a == 0) return b;
  if (b == 0) return a;
  return (a < b) ? a : b;
}

// something is in the notch, close enough to act on
static bool weightInNotch()
{
  uint16_t d = nearest(tofMM[T_FUNL], tofMM[T_FUNR]);
  return d > 0 && d < PICKUP_TRIGGER_MM;
}

// the upright check: beam broken = standing weight, clear = lying or empty
static bool weightUpright()
{
  uint16_t d = tofMM[T_UPRIGHT];
  return d >= UPRIGHT_MIN_MM && d < UPRIGHT_TRIGGER_MM;
}

// ============================================================================
//  8x8 MATRIX ToF  (front scanner)
// ============================================================================

#if USE_TOF_X8
static void x8Init()
{
  if (x8.begin() != 0) {
    Serial.println("!! SEN0628 @0x33 not responding - check the I2C cable");
    return;
  }
  if (x8.setRangingMode(eMatrix_8X8) != 0) {
    Serial.println("!! SEN0628 would not enter 8x8 mode");
    return;
  }
  x8Ok = true;
  Serial.println("SEN0628 8x8 ok");
}

static void x8Update()
{
  static unsigned long last = 0;
  if (!x8Ok || millis() - last < X8_READ_MS) return;
  last = millis();
  x8.getAllData(x8Grid);       // 64 zones, row-major, units assumed mm
}

static inline uint16_t zone(uint8_t row, uint8_t col)
{
#if X8_ROW_FLIP
  row = 7 - row;
#endif
  uint16_t v = x8Grid[row * 8 + col];
  return (v == 0 || v > X8_MAX_VALID_MM) ? 0 : v;
}

// nearest return anywhere in the working band
static uint16_t x8FrontMM()
{
  uint16_t best = 0;
  for (uint8_t r = X8_BAND_LO; r <= X8_BAND_HI; r++)
    for (uint8_t c = 0; c < 8; c++)
      best = nearest(best, zone(r, c));
  return best;
}

// bearing to the nearest thing in front, degrees, + = to the robot's right
static float x8BearingDeg(uint16_t *distOut)
{
  uint16_t best = 0;
  uint8_t bestCol = 4;
  for (uint8_t r = X8_BAND_LO; r <= X8_BAND_HI; r++)
    for (uint8_t c = 0; c < 8; c++) {
      uint16_t v = zone(r, c);
      if (v == 0) continue;
      if (best == 0 || v < best) { best = v; bestCol = c; }
    }
  if (distOut) *distOut = best;
  return ((float)bestCol - 3.5f) * X8_ZONE_DEG * X8_COL_SIGN;
}

// a wall fills every column at a similar distance; a 50mm weight does not
static bool x8WallAhead()
{
  uint16_t lo = 0, hi = 0;
  uint8_t seen = 0;
  for (uint8_t c = 0; c < 8; c++) {
    uint16_t colMin = 0;
    for (uint8_t r = X8_BAND_LO; r <= X8_BAND_HI; r++)
      colMin = nearest(colMin, zone(r, c));
    if (colMin == 0) continue;
    seen++;
    if (lo == 0 || colMin < lo) lo = colMin;
    if (colMin > hi) hi = colMin;
  }
  return (seen >= 7) && ((hi - lo) < X8_WALL_SPREAD_MM);
}

static void x8PrintGrid()
{
  if (!x8Ok) { Serial.println("  8x8 not initialised"); return; }
  Serial.println("  --- 8x8 grid (mm, 0 = no return) ---");
  for (uint8_t r = 0; r < 8; r++) {
    Serial.print("  r"); Serial.print(r); Serial.print(": ");
    for (uint8_t c = 0; c < 8; c++) {
      Serial.printf("%5u", zone(r, c));
    }
    Serial.println();
  }
  uint16_t d = 0;
  float b = x8BearingDeg(&d);
  Serial.printf("  nearest %umm at %.1f deg, wall:%s\n",
                d, b, x8WallAhead() ? "YES" : "no");
}
#else
static void x8Init() {}
static void x8Update() {}
static uint16_t x8FrontMM() { return 0; }
static float x8BearingDeg(uint16_t *d) { if (d) *d = 0; return 0.0f; }
static bool x8WallAhead() { return false; }
static void x8PrintGrid() {}
#endif

// front clearance: the array is the only forward-looking sensor now
static uint16_t frontMM() { return x8FrontMM(); }

// ============================================================================
//  IR / IMU / INDUCTIVE
// ============================================================================

static float irVolts(int pin)
{
  long sum = 0;
  for (int i = 0; i < IR_SAMPLES; i++) sum += analogRead(pin);
  return ((float)sum / IR_SAMPLES) * (ADC_VREF / ADC_MAX);
}

static uint16_t irMM(int pin)
{
  float v = irVolts(pin);
  if (v < 0.10f) return 0;
  float mm = IR_A * powf(v, IR_B);
  if (mm < IR_MIN_MM || mm > IR_MAX_MM) return 0;   // outside the rated band
  return (uint16_t)mm;
}

static void irUpdate()
{
  static unsigned long last = 0;
  if (millis() - last < IR_READ_MS) return;
  last = millis();
  irLmm = irMM(PIN_IR_L);
  irRmm = irMM(PIN_IR_R);
}

// Wall following reads the sides through these, so switching USE_SIDE_TOF
// changes the sensor without touching the control code. 0 = nothing seen.
#if USE_SIDE_TOF
static uint16_t sideLeftMM()  { return tofOk[T_SIDE_L] ? tofMM[T_SIDE_L] : 0; }
static uint16_t sideRightMM() { return tofOk[T_SIDE_R] ? tofMM[T_SIDE_R] : 0; }
#else
static uint16_t sideLeftMM()  { return irLmm; }
static uint16_t sideRightMM() { return irRmm; }
#endif

static void imuInit()
{
  Wire1.begin();
  imuOk = bno.begin(OPERATION_MODE_NDOF);
  if (!imuOk) {
    Serial.println("!! BNO055 not found on Wire1 @0x28 - check the I2C 1 cable");
    return;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ok - gyro cal must read 3 before any heading test");
}

static void imuUpdate()
{
  if (!imuOk) return;
  sensors_event_t e;
  bno.getEvent(&e);
  headingDeg = e.orientation.x;
  pitchDeg   = e.orientation.y;      // useful for ramp/speed-bump detection
  bno.getCalibration(&calSys, &calGyro, &calAcc, &calMag);
}

static float wrap180(float d)
{
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static void inductiveUpdate()
{
  bool raw = digitalRead(PIN_INDUCTIVE);
  inductiveMetal = INDUCTIVE_ACTIVE_LOW ? !raw : raw;
}

// ============================================================================
//  TEST STATE MACHINE
// ============================================================================

enum Test {
  T_IDLE, T_SENSORS, T_GRID, T_BENCH, T_RAMP, T_STRAIGHT,
  T_TURN, T_HOLD, T_WALL, T_OBSTACLE, T_APPROACH
};

static Test  test = T_IDLE;
static unsigned long testStart = 0, phaseStart = 0;
static int   phase = 0;
static float targetHeading = 0.0f, startHeading = 0.0f;
static unsigned long inTolSince = 0;
static int   rampPct = 0;

static void endTest(const char *why)
{
  stopAll();
  test = T_IDLE;
  Serial.print("--- stopped: "); Serial.println(why);
}

static void beginTest(Test t, const char *name)
{
  test = t;
  testStart = phaseStart = millis();
  phase = 0;
  startHeading = headingDeg;
  Serial.print("=== "); Serial.print(name); Serial.println("  (any key = STOP)");
}

static void printHelp()
{
  Serial.println();
  Serial.println("=========== RoboCup nav bring-up ===========");
  Serial.println(" ?  this menu");
  Serial.println(" s  sensor stream, motors idle");
  Serial.println(" a  print the 8x8 grid once (work out its orientation)");
  Serial.println(" 1  motor bench   - each side fwd/rev in turn");
  Serial.println(" 2  speed ramp    - find the deadband and the top end");
  Serial.println(" 3  straight 3s   - open loop, reports heading drift");
  Serial.println(" 4  turn +90      - closed loop on IMU, reports error");
  Serial.println(" 5  heading hold  - drive forward holding start heading");
  Serial.println(" 6  wall follow   - P control on the side IR");
  Serial.println(" 7  obstacle stop - creep forward, stop on the 8x8");
  Serial.println(" 8  approach      - steer to the nearest target, verify it");
  Serial.println(" x  STOP");
  Serial.println("============================================");
}

// ---- tests --------------------------------------------------------------

static void runBench()
{
  static const struct { int l, r; const char *msg; } steps[] = {
    { +CRUISE_PCT, 0, "LEFT forward  (left track should go forward)" },
    { -CRUISE_PCT, 0, "LEFT reverse" },
    { 0, +CRUISE_PCT, "RIGHT forward" },
    { 0, -CRUISE_PCT, "RIGHT reverse" },
    { +CRUISE_PCT, +CRUISE_PCT, "BOTH forward" },
    { 0, 0, "done" },
  };

  if (millis() - phaseStart < 1000) return;
  phaseStart = millis();

  if (steps[phase].l == 0 && steps[phase].r == 0 && phase > 0) {
    endTest("bench complete");
    return;
  }
  Serial.print("  "); Serial.println(steps[phase].msg);
  drive(steps[phase].l, steps[phase].r);
  phase++;
}

// watch for the % where it actually starts moving, and whether the top of
// the range stalls (see motor.cpp pulse-width note)
static void runRamp()
{
  if (millis() - phaseStart < 800) return;
  phaseStart = millis();

  if (rampPct > 100) { endTest("ramp complete"); return; }
  Serial.print("  "); Serial.print(rampPct); Serial.println("%");
  drive(rampPct, rampPct);
  rampPct += 10;
}

// the drift number is how mismatched the tracks are - it's what heading
// hold has to correct for
static void runStraight()
{
  if (millis() - testStart > 3000) {
    Serial.printf("  heading drift over 3s: %.1f deg\n",
                  wrap180(headingDeg - startHeading));
    endTest("straight complete");
    return;
  }
  drive(CRUISE_PCT, CRUISE_PCT);
}

static void runTurn()
{
  if (!imuOk) { endTest("no IMU"); return; }

  if (phase == 0) {
    targetHeading = headingDeg + 90.0f * HEADING_SIGN;
    phase = 1;
    inTolSince = 0;
  }

  float err = wrap180(targetHeading - headingDeg) * HEADING_SIGN;

  if (fabsf(err) < TURN_TOL_DEG) {
    if (inTolSince == 0) inTolSince = millis();
    stopAll();
    if (millis() - inTolSince > TURN_SETTLE_MS) {
      Serial.printf("  final error: %.1f deg\n", err);
      endTest("turn complete");
    }
    return;
  }
  inTolSince = 0;

  int cmd = (int)(KP_HEADING * err);
  cmd = constrain(cmd, -TURN_PCT, TURN_PCT);
  if (abs(cmd) < MIN_TURN_PCT) cmd = (cmd > 0) ? MIN_TURN_PCT : -MIN_TURN_PCT;

  drive(+cmd, -cmd);            // cmd > 0 = turn right
}

static void runHold()
{
  if (!imuOk) { endTest("no IMU"); return; }

  float err = wrap180(startHeading - headingDeg) * HEADING_SIGN;
  int corr = constrain((int)(KP_HEADING * err), -HEAD_CORR_MAX, HEAD_CORR_MAX);

  drive(CRUISE_PCT + corr, CRUISE_PCT - corr);

  uint16_t f = frontMM();
  if (f > 0 && f < FRONT_STOP_MM) endTest("obstacle ahead");
}

static void runWall()
{
  uint16_t l = sideLeftMM(), r = sideRightMM();
  int corr = 0;

  if (l > 0 && (r == 0 || l < r))       corr =  (int)(KP_WALL * (WALL_TARGET_MM - (int)l));
  else if (r > 0)                        corr = -(int)(KP_WALL * (WALL_TARGET_MM - (int)r));
  corr = constrain(corr, -WALL_CORR_MAX, WALL_CORR_MAX);

  drive(CRUISE_PCT + corr, CRUISE_PCT - corr);

  uint16_t f = frontMM();
  if (f > 0 && f < FRONT_STOP_MM) endTest("obstacle ahead");
}

// proves the 8x8 reacts fast enough at speed
static void runObstacle()
{
  uint16_t f = frontMM();
  if (f > 0 && f < FRONT_STOP_MM) {
    Serial.printf("  stopped at %umm, wall:%s\n", f, x8WallAhead() ? "YES" : "no");
    endTest("obstacle detected");
    return;
  }
  drive(CRUISE_PCT, CRUISE_PCT);
}

// the whole nav chain end to end: pick a target off the 8x8, steer onto it,
// stop when the bottom pair says it's in the notch, then report what the
// upright ToF and the inductive sensor make of it.
static void runApproach()
{
  if (weightInNotch()) {
    stopAll();
    uint16_t up = tofMM[T_UPRIGHT];
    Serial.printf("  in notch: L:%u R:%u | upright ToF:%u -> %s | inductive: %s\n",
                  tofMM[T_FUNL], tofMM[T_FUNR], up,
                  weightUpright() ? "UPRIGHT" : "lying/none",
                  inductiveMetal ? "METAL" : "non-metal");
    Serial.printf("  verdict: %s\n",
                  (weightUpright() && inductiveMetal) ? "REAL WEIGHT"
                                                      : "reject / reposition");
    endTest("approach complete");
    return;
  }

  uint16_t d = 0;
  float bearing = x8BearingDeg(&d);

  if (d == 0) { drive(0, 0); return; }            // nothing to steer at

  if (x8WallAhead() && d < FRONT_STOP_MM) {
    endTest("that's a wall, not a weight");
    return;
  }

  int corr = constrain((int)(KP_BEARING * bearing),
                       -BEARING_CORR_MAX, BEARING_CORR_MAX);
  drive(APPROACH_PCT + corr, APPROACH_PCT - corr);
}

// ---- telemetry ----------------------------------------------------------

static void printTelemetry()
{
  static unsigned long last = 0;
  if (millis() - last < TELEMETRY_MS) return;
  last = millis();

  Serial.print("ToF");
  for (int i = 0; i < T_COUNT; i++) {
    Serial.print(" "); Serial.print(TOF_CFG[i].name); Serial.print(":");
    if (!tofOk[i])          Serial.print("--");
    else if (tofMM[i] == 0) Serial.print("clr");
    else                    Serial.print(tofMM[i]);
  }

  uint16_t d = 0;
  float b = x8BearingDeg(&d);
  Serial.printf(" | X8 %umm @%.0fdeg %s", d, b, x8WallAhead() ? "WALL" : "");
  Serial.printf(" | IR L:%u R:%u", irLmm, irRmm);      // always printed, so you
  Serial.printf(" | side L:%u R:%u",                    // can compare IR vs ToF
                sideLeftMM(), sideRightMM());
  Serial.printf(" | ind:%s", inductiveMetal ? "METAL" : "-");
  Serial.printf(" | hdg:%.1f pitch:%.1f cal g%u/s%u", headingDeg, pitchDeg,
                calGyro, calSys);
  Serial.printf(" | drv %d,%d\n", lastL, lastR);
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  motor_init();                  // neutral pulses straight away
  stopAll();

  analogReadResolution(ADC_BITS);
  pinMode(PIN_IR_L, INPUT);
  pinMode(PIN_IR_R, INPUT);
  pinMode(PIN_INDUCTIVE, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("\n--- nav bring-up rig ---");
  tofInitAll();
  x8Init();
  imuInit();

  printHelp();
}

void loop()
{
  // 1. sensors first, always, whatever the test is doing
  tofUpdate();
  x8Update();
  irUpdate();
  imuUpdate();
  inductiveUpdate();

  // 2. serial: any key stops a running test; commands only work from idle
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();      // flush the rest

    if (test != T_IDLE && test != T_SENSORS) {
      endTest("key press");
    } else {
      switch (c) {
        case '?': printHelp(); break;
        case 's': test = (test == T_SENSORS) ? T_IDLE : T_SENSORS; break;
        case 'a': x8PrintGrid(); break;
        case '1': beginTest(T_BENCH,    "motor bench");    break;
        case '2': rampPct = 0;
                  beginTest(T_RAMP,     "speed ramp");     break;
        case '3': beginTest(T_STRAIGHT, "straight 3s");    break;
        case '4': beginTest(T_TURN,     "turn +90");       break;
        case '5': beginTest(T_HOLD,     "heading hold");   break;
        case '6': beginTest(T_WALL,     "wall follow");    break;
        case '7': beginTest(T_OBSTACLE, "obstacle stop");  break;
        case '8': beginTest(T_APPROACH, "approach");       break;
        case 'x': stopAll(); break;
        default:  break;
      }
    }
  }

  // 3. hard timeout - nothing drives for more than 10s unattended
  if (test != T_IDLE && test != T_SENSORS &&
      millis() - testStart > TEST_TIMEOUT_MS) {
    endTest("timeout");
  }

  // 4. reversing guard, applies to every test
  if (lastL < 0 && lastR < 0 && tofMM[T_REAR] > 0 &&
      tofMM[T_REAR] < REAR_STOP_MM) {
    endTest("rear blocked");
  }

  // 5. run the active test
  switch (test) {
    case T_BENCH:    runBench();    break;
    case T_RAMP:     runRamp();     break;
    case T_STRAIGHT: runStraight(); break;
    case T_TURN:     runTurn();     break;
    case T_HOLD:     runHold();     break;
    case T_WALL:     runWall();     break;
    case T_OBSTACLE: runObstacle(); break;
    case T_APPROACH: runApproach(); break;
    default: break;
  }

  // 6. telemetry whenever something is happening
  if (test != T_IDLE) printTelemetry();
}
*/