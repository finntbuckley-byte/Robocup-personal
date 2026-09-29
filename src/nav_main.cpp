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
#include <EEPROM.h>
#include <math.h>
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
#include "pose.h"
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

// I2C bus clear, run BEFORE Wire.begin(). Sensors keep their power across a
// Teensy reset / re-flash, so one caught mid-byte can hold SDA low forever
// and lock the bus (2026-09-28: the SEN0628 hung setup inside its first
// transfer after repeated re-flashes). Clocking SCL up to 9 times lets the
// stuck chip finish its byte, then a STOP releases the bus. Returns true if
// SDA was stuck (worth logging - it means something reset mid-transfer).
static bool i2cBusClear(uint8_t sdaPin, uint8_t sclPin)
{
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT_OPENDRAIN);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);
  bool wasStuck = (digitalRead(sdaPin) == LOW);

  for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++)
  {
    digitalWrite(sclPin, LOW);  delayMicroseconds(10);
    digitalWrite(sclPin, HIGH); delayMicroseconds(10);
  }
  // STOP: SDA low -> high while SCL is high
  pinMode(sdaPin, OUTPUT_OPENDRAIN);
  digitalWrite(sdaPin, LOW);  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH); delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(10);
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  return wasStuck;
}

// live I2C scan of both buses ('i') - who answers where right now.
// Valid 7-bit range ONLY: 0x00-0x07 and 0x78-0x7F are reserved (general
// call, CBUS, Hs-mode master codes, 10-bit prefix). Probing them rebooted
// the BNO055 every time (2026-09-28: 3/3 scans -> IMU RESET).
static void i2cScan()
{
  TwoWire *buses[2] = { &Wire, &Wire1 };
  const char *names[2] = { "Wire ", "Wire1" };
  for (int b = 0; b < 2; b++)
  {
    Serial.print(names[b]); Serial.print(":");
    for (uint8_t a = 0x08; a <= 0x77; a++)
    {
      // never send the BNO055 an empty (address-only) write - see below
      if (buses[b] == &IMU_WIRE && a == IMU_ADDR)
      {
        IMU_WIRE.beginTransmission(IMU_ADDR); IMU_WIRE.write((uint8_t)0x00);   // CHIP_ID register
        bool idOk = IMU_WIRE.endTransmission(false) == 0 &&
                    IMU_WIRE.requestFrom(IMU_ADDR, (uint8_t)1) == 1 && IMU_WIRE.read() == 0xA0;
        if (idOk) { Serial.print(" 0x"); Serial.print(a, HEX); Serial.print("(BNO055)"); }
        continue;
      }
      buses[b]->beginTransmission(a);
      if (buses[b]->endTransmission() == 0) { Serial.print(" 0x"); Serial.print(a, HEX); }
    }
    Serial.println();
  }
}          // 'x' on the serial menu - bench safety only

static void printNavTuning()
{
  NavTuning &t = navTuning();
  Serial.println("--- nav tuning (live - not saved across a reflash) ---");
  Serial.print("  rev   (reject reverse)          = "); Serial.print(t.rejectReverseMm);  Serial.println(" mm");
  Serial.print("  pdeg  (reject pivot)            = "); Serial.print(t.rejectPivotDeg);   Serial.println(" deg");
  Serial.print("  turn  (reposition pivot)        = "); Serial.print(t.repositionTurnMs); Serial.println(" ms");
  Serial.print("  disc  (lying discrepancy max)   = "); Serial.print(t.lyingDiscrepancyMm); Serial.println(" mm");
  Serial.print("  lconf (lying confirm window)    = "); Serial.print(t.lyingConfirmMs); Serial.println(" ms");
  Serial.print("  ccap  (creep cap, notch seen)   = "); Serial.print(t.creepCapMs); Serial.println(" ms");
  Serial.println("paste-ready for config.h: REJECT_REVERSE_MM / REJECT_PIVOT_DEG / REPOSITION_TURN_MS / LYING_DISCREPANCY_MM / LYING_CONFIRM_MS / CREEP_HARD_CAP_MS");
}

static void printHelp()
{
  Serial.println("\n=========== nav build ===========");
  Serial.println(" ?  this menu");
  Serial.println(" g  print the 8x8 grid (orientation / band check)");
  Serial.println(" u  live ToF line (name header + one updating line) - press again to stop");
  Serial.println(" i  I2C scan of Wire and Wire1");
  Serial.println(" t  telemetry on/off");
  Serial.println(" x  KILL - stop motors until reset");
  Serial.println(" z  zero the distance trip meter");
  Serial.println(" d  print the trip meter (encoder distance since 'z')");
  Serial.println(" l  print the last round's summary (x / y / heading - saved, survives power-off)");
  Serial.println(" live tuning - type the line, then Enter:");
  Serial.println(" :nt          print live nav tuning (reverse/turn/lying-reject/creep timings)");
  Serial.println(" :rev <mm>    set reject reverse distance (e.g. ':rev 200')");
  Serial.println(" :pdeg <deg>  set reject pivot angle (e.g. ':pdeg 90')");
  Serial.println(" :turn <ms>   set reposition pivot duration (e.g. ':turn 700')");
  Serial.println(" :disc <mm>   set lying-weight discrepancy max (e.g. ':disc 40')");
  Serial.println(" :lconf <ms>  set lying-weight confirm window (e.g. ':lconf 1500')");
  Serial.println(" :ccap <ms>   set creep cap once the notch sees something (e.g. ':ccap 8000')");
  if (GO_STOPS_ROUND) Serial.println(" GO pressed again during a round = STOP (testing; off for competition)");
  Serial.println("=================================");
}

static bool telemetryOn = true;

static void printTelemetryHeader()
{
  Serial.println("ms\tround\tmode\tBL\tBR\tUP\tTOP\tTOPok\tREAR\tX8L\tX8R\tIRL\tIRR\tfun\tind\tW\tpicks\treal\tdummy\tonb\trej\tabd\tdrvL\tdrvR\todo\todoRaw\tslip\tstall\thdg\tpx\tpy\timuMode\timuRst\ttofRec\tgoD\tgoA");
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
  Serial.print(tofTop);                 Serial.print('\t');
  Serial.print(tofOk(TOF_TOP) ? 1 : 0); Serial.print('\t');   // 0 = sensor not up (unwired/failed/stale), not "nothing seen"
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
  Serial.print(approachGiveUpCount());  Serial.print('\t');
  Serial.print(lastDriveLeftPct());     Serial.print('\t');
  Serial.print(lastDriveRightPct());    Serial.print('\t');
  Serial.print(odomDistanceMM(), 0);    Serial.print('\t');   // slip-corrected mm
  Serial.print(odomRawDistanceMM(), 0); Serial.print('\t');   // encoders only
  Serial.print(odomSlipping() ? 1 : 0); Serial.print('\t');
  Serial.print(odomStalled() ? 1 : 0);  Serial.print('\t');
  if (imuOk()) Serial.print(imuHeadingDeg(), 1); else Serial.print('-');   // + = right of start
  Serial.print('\t');
  Serial.print(poseXmm(), 0);           Serial.print('\t');   // mm forward of the start
  Serial.print(poseYmm(), 0);           Serial.print('\t');   // mm right of the start
  Serial.print("0x"); Serial.print(imuOprMode(), HEX); Serial.print('\t');   // 0x8 = IMUPLUS, 0x0 = rebooted
  Serial.print(imuResetCount());        Serial.print('\t');
  Serial.print(tofRecoverCount());      Serial.print('\t');
  // raw GO pin, digital + 10-bit analog - bring-up diagnostic
  if (PIN_GO >= 0) { Serial.print(digitalRead(PIN_GO)); Serial.print('\t'); Serial.println(analogRead(PIN_GO)); }
  else             { Serial.println("-\t-"); }
}

// ---------------------------------------------------------------------------
// Encoder accuracy check. TRIP METER: 'z' zeroes, 'd' prints distance since.
// Works in WAIT too (odomUpdate() always runs), so the robot can be pushed by
// hand along a tape measure.
// ROUND SUMMARY: at ROUND OVER (timer or GO-stop) x / y / heading / distance
// are printed AND saved to EEPROM, so a round can run untethered: plug in
// afterwards and press 'l' (it's also printed at every boot).
// ---------------------------------------------------------------------------
static float tripRawStart = 0, tripCorrStart = 0;
static long  tripLStart = 0, tripRStart = 0;

static void tripZero()
{
  tripRawStart = odomRawDistanceMM(); tripCorrStart = odomDistanceMM();
  tripLStart = encLeftCount(); tripRStart = encRightCount();
  Serial.println(">>> trip zeroed");
}

static void tripPrint()
{
  Serial.println("TRIP\traw_mm\tcorr_mm\tencL\tencR\tslipEv");
  Serial.print("TRIP\t");
  Serial.print(odomRawDistanceMM() - tripRawStart, 0); Serial.print('\t');
  Serial.print(odomDistanceMM() - tripCorrStart, 0);   Serial.print('\t');
  Serial.print(encLeftCount() - tripLStart);            Serial.print('\t');
  Serial.print(encRightCount() - tripRStart);           Serial.print('\t');
  Serial.println(odomSlipEvents());
}

static float wrap180(float d)
{
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// unwrapped heading (total rotation, can pass +/-180) and path length (every
// mm driven, forward or back) - accumulated while the round runs
static float hdgUnwrapped = 0, hdgLast = 0;
static float pathMM = 0, pathLastRaw = 0;

static void roundTrackReset()
{
  hdgUnwrapped = 0; hdgLast = imuHeadingDeg();
  pathMM = 0; pathLastRaw = odomRawDistanceMM();
}

static void roundTrackUpdate()
{
  float h = imuHeadingDeg();
  hdgUnwrapped += wrap180(h - hdgLast);
  hdgLast = h;
  float raw = odomRawDistanceMM();
  pathMM += fabsf(raw - pathLastRaw);
  pathLastRaw = raw;
}

const uint32_t SUMMARY_MAGIC = 0x52533233;   // "RS23" - change it if the struct changes
const int      SUMMARY_EEPROM_ADDR = 64;     // clear of the old IMU-calibration sketch's bytes
struct RoundSummary
{
  uint32_t magic;
  uint32_t seq;          // counts up every saved round
  uint32_t elapsedMs;
  float xMM, yMM;        // pose.h: +x = facing at GO, +y = right
  float hdgDeg;          // final heading, -180..180 (+ = right of start)
  float hdgTotalDeg;     // unwrapped: net rotation including full turns
  float netRawMM, netCorrMM, pathMM;
  uint16_t slipEvents;
  uint8_t  imuOk, onboard;
  uint16_t imuResets, rejects;
};

static void summaryPrint(const RoundSummary &r, const char *title)
{
  Serial.print("--- "); Serial.print(title); Serial.print(" (round #"); Serial.print(r.seq); Serial.println(") ---");
  Serial.println("SUM\tt_s\tx_mm\ty_mm\thdg_deg\thdgTot_deg\tnetRaw_mm\tnetCorr_mm\tpath_mm\tslipEv\timuOk\timuRst\tonb\trej");
  Serial.print("SUM\t");
  Serial.print(r.elapsedMs / 1000.0f, 1); Serial.print('\t');
  Serial.print(r.xMM, 0);         Serial.print('\t');
  Serial.print(r.yMM, 0);         Serial.print('\t');
  Serial.print(r.hdgDeg, 1);      Serial.print('\t');
  Serial.print(r.hdgTotalDeg, 1); Serial.print('\t');
  Serial.print(r.netRawMM, 0);    Serial.print('\t');
  Serial.print(r.netCorrMM, 0);   Serial.print('\t');
  Serial.print(r.pathMM, 0);      Serial.print('\t');
  Serial.print(r.slipEvents);     Serial.print('\t');
  Serial.print(r.imuOk);          Serial.print('\t');
  Serial.print(r.imuResets);      Serial.print('\t');
  Serial.print(r.onboard);        Serial.print('\t');
  Serial.println(r.rejects);
}

static void summaryPrintSaved()
{
  RoundSummary r;
  EEPROM.get(SUMMARY_EEPROM_ADDR, r);
  if (r.magic != SUMMARY_MAGIC) { Serial.println("(no saved round summary)"); return; }
  summaryPrint(r, "LAST SAVED ROUND");
}

static void summarySaveAndPrint()
{
  RoundSummary old;
  EEPROM.get(SUMMARY_EEPROM_ADDR, old);
  RoundSummary r;
  r.magic = SUMMARY_MAGIC;
  r.seq = (old.magic == SUMMARY_MAGIC) ? old.seq + 1 : 1;
  r.elapsedMs = roundElapsedMs();
  r.xMM = poseXmm(); r.yMM = poseYmm();
  r.hdgDeg = imuHeadingDeg(); r.hdgTotalDeg = hdgUnwrapped;
  r.netRawMM = odomRawDistanceMM(); r.netCorrMM = odomDistanceMM(); r.pathMM = pathMM;
  r.slipEvents = (uint16_t)odomSlipEvents();
  r.imuOk = imuOk() ? 1 : 0; r.onboard = (uint8_t)targetsOnBoard();
  r.imuResets = (uint16_t)imuResetCount(); r.rejects = (uint16_t)rejectedCount();
  EEPROM.put(SUMMARY_EEPROM_ADDR, r);   // once per round, motors already stopped
  summaryPrint(r, "ROUND SUMMARY (saved - 'l' reprints it)");
}

// Never blocks (the old readStringUntil('\n') froze loop() for up to 1 s on
// a key sent without Enter - motors held their last command, no avoidance).
// Single keys act the moment they arrive, as before. Live tuning lines start
// with ':' and run on Enter (":rev 500") - the prefix keeps 't' (telemetry)
// apart from ":turn". 'x' kills immediately, even mid-line.
static void runTuningLine(char *line)
{
  char *arg = strchr(line, ' ');
  if (arg) *arg++ = 0;
  long v = arg ? atol(arg) : -1;
  NavTuning &t = navTuning();

  if      (!strcasecmp(line, "nt")) {}
  else if (v < 0) { Serial.println("?? needs a value, e.g. ':rev 500'"); return; }
  else if (!strcasecmp(line, "rev"))   t.rejectReverseMm    = (int)v;
  else if (!strcasecmp(line, "pdeg"))  t.rejectPivotDeg     = (int)v;
  else if (!strcasecmp(line, "turn"))  t.repositionTurnMs   = (unsigned long)v;
  else if (!strcasecmp(line, "disc"))  t.lyingDiscrepancyMm = (int)v;
  else if (!strcasecmp(line, "lconf")) t.lyingConfirmMs     = (unsigned long)v;
  else if (!strcasecmp(line, "ccap"))  t.creepCapMs         = (unsigned long)v;
  else { Serial.print("?? unknown ':"); Serial.print(line); Serial.println("' - '?' for the menu"); return; }
  printNavTuning();
}

static void handleSerial()
{
  static char buf[24];
  static uint8_t n = 0;
  static bool inLine = false;

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == 'x') { killed = true; driveHardStop(); Serial.println("!!! KILLED - reset to run again"); inLine = false; n = 0; continue; }

    if (inLine)
    {
      if (c == '\r' || c == '\n') { buf[n] = 0; inLine = false; n = 0; runTuningLine(buf); }
      else if (n < sizeof(buf) - 1) buf[n++] = c;
      continue;
    }

    switch (c)
    {
      case ':': inLine = true; n = 0; break;
      case '?': printHelp(); break;
      case 'g': x8PrintGrid(); break;
      case 'u': tofPrintRaw(); break;
      case 'i': i2cScan(); probeTofs("now"); break;
      case 'z': tripZero(); break;
      case 'd': tripPrint(); break;
      case 'l': summaryPrintSaved(); break;
      case 't': telemetryOn = !telemetryOn; if (telemetryOn) printTelemetryHeader(); break;
      default: break;   // stray Enter etc.
    }
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

  if (i2cBusClear(I2C0_SDA_PIN, I2C0_SCL_PIN)) Serial.println("!! Wire (I2C0) was stuck - bus cleared");
  if (i2cBusClear(I2C1_SDA_PIN, I2C1_SCL_PIN)) Serial.println("!! Wire1 (I2C1) was stuck - bus cleared");
  Wire.begin();  Wire.setClock(400000);
  Wire1.begin(); Wire1.setClock(400000);

  Serial.println("\n--- RoboCup G23 nav build ---");
  tofInit();
  x8Init();              // blocks ~5s setting 8x8 mode
  irSensorsInit();
  odomInit();
  Serial.println(imuInit() ? "IMU ok (BNO055, IMUPLUS)" : "!! IMU not found - no heading hold");
  funnelSensorInit();
  roundInit();

  printHelp();
  printNavTuning();
  summaryPrintSaved();
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
  poseUpdate();          // x/y from the start (logging only for now)

  // 2. weight candidate
  weightDetectUpdate();

  // 3. crane FSM - always ticks, navigation just starts/watches it
  collection_update();
  gate_update();

  // 4. sorting - independent of navigation/collection
  funnelSortUpdate();

  // 5. round + navigation
  roundUpdate();
  if (roundJustStarted()) { navigationInit(); odomReset(); imuZero(); headingHoldReset(); poseReset(); roundTrackReset(); }   // distance + heading from the start position

  // round summary: tracked while running, saved + printed once at ROUND OVER
  static bool wasRunning = false;
  if (roundRunning()) roundTrackUpdate();
  if (wasRunning && roundOver()) summarySaveAndPrint();
  wasRunning = roundRunning();

  if (killed)             driveHardStop();
  else if (!roundRunning()) stopMotors();
  else                           navigationUpdate();

  // 6. telemetry
  printTelemetry();
  tofPrintRawTick();
}

#endif // NAV_BUILD
