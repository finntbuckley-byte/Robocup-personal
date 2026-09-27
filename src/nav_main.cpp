/* ============================================================================
 *  nav_main.cpp  -  full navigation build: seek, pick up, sort, round timer.
 *
 *  Build + upload:  pio run -e nav -t upload
 *  Serial monitor:  pio device monitor -b 115200     (press '?' for the menu)
 *
 *  Only compiled when NAV_BUILD is defined ([env:nav] sets it), so it never
 *  clashes with the other envs' setup()/loop().
 *
 *  LOOP STRUCTURE (nothing here blocks except the 8x8 read, ~20-40ms every
 *  X8_READ_MS - see x8.cpp):
 *    1. freshen every sensor (ToF chain, 8x8, side IR)
 *    2. fuse bottom ToFs + 8x8 into a stable weight candidate
 *    3. collection_update() - always, so the crane FSM progresses whatever
 *       navigation is doing
 *    4. funnelSortUpdate() - always and independently (funnel_sensor.h)
 *    5. round: WAITING -> motors stopped; RUNNING -> navigationUpdate();
 *       OVER -> motors stopped for good
 *    6. telemetry - tab-separated, paste straight into a spreadsheet
 *
 *  NOT IN THIS BUILD YET:
 *    - gate/flap SORTING logic (gate.cpp is initialised and serviced, but
 *      nothing commands it yet)
 *    - IMU / RETURN_HOME (see IMU HOOK in navigation.cpp).
 * ============================================================================ */
#ifdef NAV_BUILD

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motor.h"
#include "drive.h"
#include "collection.h"
#include "tof.h"
#include "x8.h"
#include "ir_sensors.h"
#include "funnel_sensor.h"
#include "weight_detect.h"
#include "navigation.h"
#include "round.h"
#include "gate.h"
#include "odometry.h"
#include "imu.h"

static bool killed = false;

// TEMP DIAGNOSTIC (28/9, notch ToF falling back to 0x29): which of the ToF
// addresses answer, and what model sits at 0x29. Remove once found.
static bool ping(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
static void probeTofs(const char *when)
{
  Serial.print("[probe "); Serial.print(when); Serial.print("] answering:");
  for (uint8_t a : { (uint8_t)0x29, (uint8_t)0x34, (uint8_t)0x35, (uint8_t)0x36, (uint8_t)0x37 })
    if (ping(a)) { Serial.print(" 0x"); Serial.print(a, HEX); }
  if (ping(0x29))
  {
    Wire.beginTransmission(0x29); Wire.write(0x01); Wire.write(0x0F);   // VL53L1X model ID reg
    Wire.endTransmission(false); Wire.requestFrom((uint8_t)0x29, (uint8_t)1);
    int id = Wire.available() ? Wire.read() : -1;
    Serial.print("  | 0x29 model id 0x"); Serial.print(id, HEX);
    Serial.print(id == 0xEA ? " = VL53L1X" : " = not an L1X");
  }
  Serial.println();
}

// live I2C scan of both buses ('i') - who answers where right now
static void i2cScan()
{
  TwoWire *buses[2] = { &Wire, &Wire1 };
  const char *names[2] = { "Wire ", "Wire1" };
  for (int b = 0; b < 2; b++)
  {
    Serial.print(names[b]); Serial.print(":");
    for (uint8_t a = 1; a < 127; a++)
    {
      buses[b]->beginTransmission(a);
      if (buses[b]->endTransmission() == 0) { Serial.print(" 0x"); Serial.print(a, HEX); }
    }
    Serial.println();
  }
}          // 'x' on the serial menu - bench safety only

static void printHelp()
{
  Serial.println("\n=========== nav build ===========");
  Serial.println(" ?  this menu");
  Serial.println(" g  print the 8x8 grid (orientation / band check)");
  Serial.println(" u  ToF diagnostics: raw mm, status, data age");
  Serial.println(" i  I2C scan of Wire and Wire1");
  Serial.println(" t  telemetry on/off");
  Serial.println(" x  KILL - stop motors until reset");
  if (GO_STOPS_ROUND) Serial.println(" GO pressed again during a round = STOP (testing; off for competition)");
  Serial.println("=================================");
}

static bool telemetryOn = true;

static void printTelemetryHeader()
{
  Serial.println("ms\tround\tmode\tBL\tBR\tUP\tREAR\tX8L\tX8R\tIRL\tIRR\tfun\tind\tW\tpicks\treal\tdummy\tonb\trej\tdrvL\tdrvR\todo\todoRaw\tslip\tstall\thdg\tgoD\tgoA");
}

static void printTelemetry()
{
  static unsigned long last = 0;
  if (!telemetryOn || millis() - last < TELEMETRY_MS) return;
  last = millis();

  Serial.print(roundElapsedMs());       Serial.print('\t');
  Serial.print(roundPhaseName());       Serial.print('\t');
  Serial.print(roundRunning() ? modeName() : "-"); Serial.print('\t');
  Serial.print(tofBL);                  Serial.print('\t');
  Serial.print(tofBR);                  Serial.print('\t');
  Serial.print(tofUpright);             Serial.print('\t');
  Serial.print(tofRear);                Serial.print('\t');
  if (x8Fresh()) { Serial.print(x8LeftMM()); Serial.print('\t'); Serial.print(x8RightMM()); }
  else           { Serial.print("stale\tstale"); }
  Serial.print('\t');
  Serial.print(irSideLMM);              Serial.print('\t');
  Serial.print(irSideRMM);              Serial.print('\t');
  Serial.print(weightInFunnel() ? 1 : 0);    Serial.print('\t');
  Serial.print(inductiveMetalNow() ? 1 : 0); Serial.print('\t');
  if (weightFound)
  {
    Serial.print(weightDistMM);
    Serial.print(weightSide == 0 ? "C" : (weightSide < 0 ? "L" : "R"));
  }
  else Serial.print(targetSuppressed() ? "supp" : "-");
  Serial.print('\t');
  Serial.print(pickupAttempts);         Serial.print('\t');
  Serial.print(realWeightCount);        Serial.print('\t');
  Serial.print(dummyCount);             Serial.print('\t');
  Serial.print(targetsOnBoard());       Serial.print('\t');
  Serial.print(rejectedCount());        Serial.print('\t');
  Serial.print(lastDriveLeftPct());     Serial.print('\t');
  Serial.print(lastDriveRightPct());    Serial.print('\t');
  Serial.print(odomDistanceMM(), 0);    Serial.print('\t');   // slip-corrected mm
  Serial.print(odomRawDistanceMM(), 0); Serial.print('\t');   // encoders only
  Serial.print(odomSlipping() ? 1 : 0); Serial.print('\t');
  Serial.print(odomStalled() ? 1 : 0);  Serial.print('\t');
  if (imuOk()) Serial.print(imuHeadingDeg(), 1); else Serial.print('-');   // + = right of start
  Serial.print('\t');
  // raw GO pin, digital + 10-bit analog - bring-up diagnostic
  if (PIN_GO >= 0) { Serial.print(digitalRead(PIN_GO)); Serial.print('\t'); Serial.println(analogRead(PIN_GO)); }
  else             { Serial.println("-\t-"); }
}

static void handleSerial()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  switch (c)
  {
    case '?': printHelp(); break;
    case 'g': x8PrintGrid(); break;
    case 'u': tofPrintRaw(); break;
    case 'i': i2cScan(); probeTofs("now"); break;
    case 'r':
      Serial.print("re-init notch ToF: "); Serial.println(tofReinitL1X(TOF_UPRIGHT) ? "ok" : "FAILED");
      probeTofs("just after re-init");
      break;
    case 't': telemetryOn = !telemetryOn; if (telemetryOn) printTelemetryHeader(); break;
    case 'x': killed = true; driveHardStop(); Serial.println("!!! KILLED - reset to run again"); break;
    default: break;
  }
}

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't wait forever - no USB at competition

  motor_init();          // neutral pulses straight away
  stopMotors();
  collection_init();     // crane parked at AT_REST, magnets off
  gate_init();           // Herkulex flap on Serial2 (CON66)

  Wire.begin();  Wire.setClock(400000);
  Wire1.begin(); Wire1.setClock(400000);

  Serial.println("\n--- RoboCup G23 nav build ---");
  tofInit();
  probeTofs("after tofInit");
  x8Init();              // blocks ~5s setting 8x8 mode
  probeTofs("after x8Init");
  irSensorsInit();
  odomInit();
  Serial.println(imuInit() ? "IMU ok (BNO055, IMUPLUS)" : "!! IMU not found - no heading hold");
  probeTofs("after imuInit");
  funnelSensorInit();
  roundInit();

  printHelp();
  printTelemetryHeader();
}

void loop()
{
  handleSerial();

  // 1. sensors
  tofUpdate();
  x8Update();
  irSensorsUpdate();
  odomUpdate();          // after the 8x8 / rear ToF - the slip check reads them
  imuUpdate();

  // 2. weight candidate
  weightDetectUpdate();

  // 3. crane FSM - always ticks, navigation just starts/watches it
  collection_update();
  gate_update();

  // 4. sorting - independent of navigation/collection
  funnelSortUpdate();

  // 5. round + navigation
  roundUpdate();
  if (roundJustStarted()) { navigationInit(); odomReset(); imuZero(); headingHoldReset(); }   // distance + heading from the start position

  if (killed)             driveHardStop();
  else if (!roundRunning()) stopMotors();
  else                           navigationUpdate();

  // 6. telemetry
  printTelemetry();
}

#endif // NAV_BUILD
