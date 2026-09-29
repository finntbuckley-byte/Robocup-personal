// stack_test.cpp - floor/bench test rig. Drive commands are real (the robot
// moves if it's on the floor; on blocks/off its tracks, nothing will
// physically move but the state prints and drv_l/drv_r telemetry still show
// intent). Runs the REAL collection.cpp crane FSM in PICKUP - no copied
// timings.
//
// SEARCH  drive forward, arc toward whichever outer weight-search ToF sees
//         something within range. -> CREEP once the underside notch ToF
//         sees something (entered its FoV).
// CREEP   drive straight. Confirm against the top ToF (patiently - see
//         config.h for why this can't be a short timeout): a confirmed
//         MISMATCH -> REVERSE. Otherwise just keep creeping - there is no
//         separate "matched, go pick it up" branch here.
// (any state) INDUCTIVE OVERRIDE - the instant it reads metal, everything
//         else is skipped and collection starts immediately.
// PICKUP  runs the real crane. A missed grab (metal still at the notch)
//         retries in place up to MAX_PICKUP_TRIES before giving up, same as
//         the main nav build.
// REVERSE reverse + pivot, then back to SEARCH for the next candidate.
//
// Serial: x = stop now | r = re-arm (from WAIT/DONE) | s = print settings
//         e<mm> max |notch-top| to still count as agreement (secondary signal)
//         f<ms> mismatch confirm time | g<mm> search range
//         h<mm> notch presence ceiling | i<mm> top presence ceiling
// Standalone test module: does NOT touch tof.cpp / navigation.cpp.
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include "config.h"
#include "motor.h"
#include "drive.h"
#include "collection.h"

// ---- SHIM: only place that touches project drive/collection code ----------
static void hwInit() {
  motor_init();
  collection_init();
}
static void hwUpdate()            { collection_update(); }
static void hwDrive(int l, int r) { drive(l, r); }
static void hwStop()              { driveHardStop(); }   // kill / stale / timeout
static void hwSoftStop()          { stopMotors(); }      // must be called EVERY loop
static void hwPickupStart()       { collection_start(); }
static bool hwPickupBusy()        { return collection_busy(); }
// ---------------------------------------------------------------------------

struct TofL1 {
  VL53L1X s;
  bool ok = false;
  uint16_t mm = 0;
  unsigned long lastMs = 0;
};
struct TofL0 {
  VL53L0X s;
  bool ok = false;
  uint16_t mm = 0;
  unsigned long lastMs = 0;
};
static TofL1 notch, top;          // L1X pair - weight discrimination
static TofL0 searchL, searchR;    // L0X pair - outer weight search

enum State { ST_WAIT, ST_SEARCH, ST_CREEP, ST_REVERSE, ST_SETTLE, ST_PICKUP, ST_DONE };
static State state = ST_WAIT;
static unsigned long stateMs = 0;
static int pickupTries = 0;   // this approach's attempt count - reset fresh each SETTLE->PICKUP

// runtime-tunable (start from config.h) - see header comment for meaning
static int dMax           = STACK_DISCREPANCY_MM;
static unsigned long mismatchMs = STACK_MISMATCH_CONFIRM_MS;
static int searchMaxMm    = STACK_SEARCH_MAX_MM;
static int notchMaxMm     = STACK_NOTCH_MAX_MM;
static int topMaxMm       = STACK_TOP_MAX_MM;

static const char* stateName(State s) {
  switch (s) {
    case ST_WAIT:    return "WAIT";
    case ST_SEARCH:  return "SEARCH";
    case ST_CREEP:   return "CREEP";
    case ST_REVERSE: return "REVERSE";
    case ST_SETTLE:  return "SETTLE";
    case ST_PICKUP:  return "PICKUP";
    default:         return "DONE";
  }
}

static void setState(State s, const char *why) {
  Serial.print(">>> "); Serial.print(stateName(state)); Serial.print(" -> "); Serial.print(stateName(s));
  Serial.print("  ("); Serial.print(why); Serial.println(')');
  state = s; stateMs = millis();
}

// ---- SX1509 XSHUT + sensor bring-up (same approach as tof.cpp) -------------
static bool sxWrite(uint8_t reg, uint8_t val) {
  SX_WIRE.beginTransmission(SX1509_ADDRESS);
  SX_WIRE.write(reg); SX_WIRE.write(val);
  return SX_WIRE.endTransmission() == 0;
}

static bool bringUpL1(TofL1 &t, int xshut, uint8_t addr, uint8_t &mask, int roi) {
  mask |= (1 << xshut);
  sxWrite(SX_REG_DATA_A, mask);
  delay(TOF_BOOT_MS);
  t.s = VL53L1X();                          // fresh object = default 0x29
  t.s.setBus(&TOF_WIRE);
  t.s.setTimeout(100);
  if (!t.s.init()) {
    mask &= ~(1 << xshut);                  // keep it off 0x29 for the next sensor
    sxWrite(SX_REG_DATA_A, mask);
    return false;
  }
  t.s.setAddress(addr);
  t.s.setDistanceMode(VL53L1X::Short);
  t.s.setMeasurementTimingBudget(TOF_L1X_BUDGET_US);
  if (roi < 16) t.s.setROISize(roi, roi);   // TODO(verify): compiles on Pololu 1.3.1
  t.s.startContinuous(TOF_PERIOD_MS);
  t.ok = true;
  return true;
}

static bool bringUpL0(TofL0 &t, int xshut, uint8_t addr, uint8_t &mask) {
  mask |= (1 << xshut);
  sxWrite(SX_REG_DATA_A, mask);
  delay(TOF_BOOT_MS);
  t.s = VL53L0X();
  t.s.setBus(&TOF_WIRE);
  t.s.setTimeout(100);
  if (!t.s.init()) {
    mask &= ~(1 << xshut);
    sxWrite(SX_REG_DATA_A, mask);
    return false;
  }
  t.s.setAddress(addr);
  t.s.startContinuous(TOF_PERIOD_MS);
  t.ok = true;
  return true;
}

static const uint8_t L0X_REG_INTERRUPT_STATUS = 0x13;
static const uint16_t TOF_NO_TARGET_MM = 8000;   // matches tof.cpp's convention

static void tofReadL1(TofL1 &t, unsigned long now) {
  if (!t.ok) return;
  if (t.s.dataReady()) {
    uint16_t mm = t.s.read(false);
    bool valid = (t.s.ranging_data.range_status == VL53L1X::RangeValid) && mm <= TOF_MAX_VALID_MM;
    t.mm = valid ? mm : 0;
    t.lastMs = now;
  }
  if (!t.lastMs || now - t.lastMs > TOF_STALE_MS) t.mm = 0;   // never act on a frozen value
}
static void tofReadL0(TofL0 &t, unsigned long now) {
  if (!t.ok) return;
  if ((t.s.readReg(L0X_REG_INTERRUPT_STATUS) & 0x07) != 0) {
    uint16_t mm = t.s.readRangeContinuousMillimeters();
    t.mm = (mm >= TOF_NO_TARGET_MM) ? 0 : mm;
    t.lastMs = now;
  }
  if (!t.lastMs || now - t.lastMs > TOF_STALE_MS) t.mm = 0;
}
static bool freshL1(const TofL1 &t, unsigned long now) { return t.ok && t.lastMs && now - t.lastMs <= TOF_STALE_MS; }
static bool freshL0(const TofL0 &t, unsigned long now) { return t.ok && t.lastMs && now - t.lastMs <= TOF_STALE_MS; }

// ---- inductive, debounced (metal must read steadily) -----------------------
static bool inductiveMetal(unsigned long now) {
  static unsigned long since = 0;
  bool raw = (digitalRead(PIN_INDUCTIVE) == (INDUCTIVE_ACTIVE_LOW ? LOW : HIGH));
  if (!raw) { since = 0; return false; }
  if (!since) since = now;
  return now - since >= INDUCTIVE_CONFIRM_MS;
}

// ---- GO button: arms only after being seen released (project rule) ---------
static bool goEdge(unsigned long now) {
  if (PIN_GO < 0) {                          // auto-start fallback
    static bool fired = false;
    if (!fired && now >= AUTO_START_DELAY_MS) { fired = true; return true; }
    return false;
  }
  static bool armed = false, stable = false, lastRaw = false;
  static unsigned long changeMs = 0;
  bool raw = (digitalRead(PIN_GO) == (GO_ACTIVE_LOW ? LOW : HIGH));
  if (raw != lastRaw) { lastRaw = raw; changeMs = now; }
  if (now - changeMs >= GO_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (!stable) armed = true;               // seen released
    else if (armed) { armed = false; return true; }
  }
  return false;
}

static void printSettings() {
  Serial.print("discrepancy max "); Serial.print(dMax); Serial.print("mm, mismatch confirm ");
  Serial.print(mismatchMs); Serial.print("ms, search range "); Serial.print(searchMaxMm); Serial.println("mm");
  Serial.print("notch max "); Serial.print(notchMaxMm); Serial.print("mm, top max ");
  Serial.print(topMaxMm); Serial.println("mm (presence ceilings - reject far-field returns)");
}

static void handleSerial() {
  static char buf[12]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') { hwStop(); setState(ST_DONE, "kill (serial)"); n = 0; continue; }   // immediate
    if (c == '\r') continue;
    if (c != '\n') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0; n = 0;
    int v = atoi(buf + 1);
    switch (buf[0]) {
      case 'r': setState(ST_WAIT, "re-armed"); break;
      case 's': break;
      case 'e': dMax = v; break;
      case 'f': mismatchMs = v; break;
      case 'g': searchMaxMm = v; break;
      case 'h': notchMaxMm = v; break;
      case 'i': topMaxMm = v; break;
      default: continue;
    }
    printSettings();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  TOF_WIRE.begin();
  TOF_WIRE.setClock(400000);                // TODO(verify): match nav_main's bus speed
  hwInit();
  pinMode(PIN_INDUCTIVE, INPUT);
  if (PIN_GO >= 0) pinMode(PIN_GO, INPUT);

  Serial.println("\n!! Drive commands are real - the robot WILL move if it's on the floor.");
  Serial.println("!! Watch the state-change prints and drv_l/drv_r telemetry to follow along.\n");

  // all XSHUT low first; anything not brought up here stays in shutdown
  bool sx = sxWrite(SX_REG_DATA_A, 0x00) && sxWrite(SX_REG_DIR_A, 0x00);
  if (!sx) { Serial.println("!! SX1509 (0x3F) not found - check SX_WIRE and CON26 cable"); return; }
  uint8_t mask = 0;
  delay(10);
  bool a = bringUpL1(notch,   STACK_XSHUT_NOTCH,    TOF_ADDRESS_START + 0, mask, 16);
  bool b = bringUpL1(top,     STACK_XSHUT_TOP,      TOF_ADDRESS_START + 1, mask, STACK_TOP_ROI);
  bool c = bringUpL0(searchL, STACK_XSHUT_SEARCH_L, TOF_ADDRESS_START + 2, mask);
  bool d = bringUpL0(searchR, STACK_XSHUT_SEARCH_R, TOF_ADDRESS_START + 3, mask);
  Serial.println("BOOT");
  Serial.print("notch   (XSHUT"); Serial.print(STACK_XSHUT_NOTCH);    Serial.println(a ? ") ok" : ") FAILED - is it an L1X?");
  Serial.print("top     (XSHUT"); Serial.print(STACK_XSHUT_TOP);     Serial.println(b ? ") ok" : ") FAILED - is it an L1X?");
  Serial.print("searchL (XSHUT"); Serial.print(STACK_XSHUT_SEARCH_L); Serial.println(c ? ") ok" : ") FAILED - is it an L0X?");
  Serial.print("searchR (XSHUT"); Serial.print(STACK_XSHUT_SEARCH_R); Serial.println(d ? ") ok" : ") FAILED - is it an L0X?");
  printSettings();
  Serial.println("ms\tstate\tnotch_mm\ttop_mm\tsL_mm\tsR_mm\tmetal\tdiff\tmismatch\tdrv_l\tdrv_r");
}

void loop() {
  unsigned long now = millis();
  unsigned long held = now - stateMs;
  hwUpdate();
  tofReadL1(notch, now);
  tofReadL1(top, now);
  tofReadL0(searchL, now);
  tofReadL0(searchR, now);
  handleSerial();

  bool metal = inductiveMetal(now);
  bool go    = goEdge(now);

  // both are long-range-capable L1X sensors - a bare "mm > 0" latches onto
  // the far wall/bare floor on open ground, so a ceiling is required to
  // treat that as "nothing" rather than a permanent false presence (2026-09-29
  // arena finding: this fired REVERSE while just driving forward)
  bool notchPresent = notch.mm > 0 && notch.mm < notchMaxMm;
  bool topPresent   = top.mm   > 0 && top.mm   < topMaxMm;
  int  diff = (notchPresent && topPresent) ? (int)notch.mm - (int)top.mm : -1;
  if (diff < 0 && notchPresent && topPresent) diff = -diff;

  // "mismatch" = notch sees something but top has NEVER confirmed (the real
  // lying-weight signature - see config.h), OR both valid but far apart (a
  // weaker secondary signal). Either resets the instant top produces ANY
  // valid reading - that's active evidence the weight is still arriving.
  bool mismatchNow = notchPresent && (!topPresent || diff > dMax);
  static unsigned long mismatchSince = 0;
  if (mismatchNow) { if (!mismatchSince) mismatchSince = now; }
  else mismatchSince = 0;
  bool mismatchConfirmed = mismatchSince && now - mismatchSince >= mismatchMs;

  bool searchOk = freshL0(searchL, now) && freshL0(searchR, now);
  bool creepOk  = freshL1(notch, now) && freshL1(top, now);

  // ---- top-level overrides, apply in every active state -------------------
  if (go && GO_STOPS_ROUND && state != ST_WAIT && state != ST_DONE) {
    hwStop(); setState(ST_DONE, "GO pressed again - stopped");
  }
  else if (metal && state != ST_WAIT && state != ST_DONE && state != ST_SETTLE && state != ST_PICKUP) {
    hwSoftStop(); setState(ST_SETTLE, "INDUCTIVE OVERRIDE - metal detected, collection now");
  }
  else switch (state) {
    case ST_WAIT:
    case ST_DONE:
      if (go) {
        if (!searchOk) Serial.println("!! a search ToF is stale/failed - not starting");
        else setState(ST_SEARCH, "GO");
      }
      break;

    case ST_SEARCH:
    {
      if (!searchOk) { hwStop(); setState(ST_DONE, "search ToF stale - stopped"); break; }
      if (notchPresent) { setState(ST_CREEP, "weight entered notch FoV"); break; }

      bool leftSees  = searchL.mm > 0 && searchL.mm < searchMaxMm;
      bool rightSees = searchR.mm > 0 && searchR.mm < searchMaxMm;
      int steer = 0;
      if (leftSees && rightSees) steer = (searchL.mm < searchR.mm) ? -STACK_SEARCH_ARC_PCT : STACK_SEARCH_ARC_PCT;
      else if (leftSees)         steer = -STACK_SEARCH_ARC_PCT;
      else if (rightSees)        steer =  STACK_SEARCH_ARC_PCT;
      hwDrive(STACK_SEARCH_SPEED_PCT + steer, STACK_SEARCH_SPEED_PCT - steer);
      break;
    }

    case ST_CREEP:
      if (!creepOk) { hwStop(); setState(ST_DONE, "notch/top ToF stale - stopped"); break; }
      if (!notchPresent) { setState(ST_SEARCH, "candidate left the notch FoV"); break; }
      if (mismatchConfirmed) { hwStop(); setState(ST_REVERSE, "mismatch confirmed - notch sees it, top never did (likely lying)"); break; }
      if (held > STACK_CREEP_MAX_MS) { hwStop(); setState(ST_REVERSE, "creep hard cap - giving up regardless"); break; }
      hwDrive(CREEP_SPEED_PCT, CREEP_SPEED_PCT);
      break;

    case ST_REVERSE:
      if (held < REJECT_REVERSE_MS) hwDrive(-REJECT_REVERSE_PCT, -REJECT_REVERSE_PCT);
      else if (held < REJECT_REVERSE_MS + REPOSITION_TURN_MS) hwDrive(REPOSITION_SPEED_PCT, -REPOSITION_SPEED_PCT);
      else setState(ST_SEARCH, "reverse complete - resuming search");
      break;

    case ST_SETTLE:   // soft stop ramps down; crane only starts once it has finished
      hwSoftStop();
      if (held >= DRIVE_DECEL_MS + 100) {
        pickupTries = 1;
        hwPickupStart();
        setState(ST_PICKUP, "crane start");
      }
      break;

    case ST_PICKUP:
      hwSoftStop();   // keeps the tracks held at zero while the crane runs
      // 100 ms guard: busy may only go true after the next collection_update()
      if ((held > 100 && !hwPickupBusy()) || held > PICKUP_TIMEOUT_MS) {
        bool timedOut   = held > PICKUP_TIMEOUT_MS;
        bool stillThere = metal;   // metal still at the notch = the grab missed

        Serial.print(">>> pickup try "); Serial.print(pickupTries); Serial.print('/'); Serial.print(MAX_PICKUP_TRIES);
        Serial.print(" - ");
        if (timedOut)        Serial.println("TIMED OUT waiting for crane");
        else if (stillThere) Serial.println("MISSED - metal still at the notch");
        else                 Serial.println("OK - metal gone, collected");

        // matches the main nav build's MAX_PICKUP_TRIES retry - a missed
        // grab tries again in place instead of abandoning a real weight
        // that's still sitting right there
        if (!timedOut && stillThere && pickupTries < MAX_PICKUP_TRIES) {
          Serial.println(">>> retrying pickup");
          pickupTries++;
          hwPickupStart();
          stateMs = now;   // restart the settle/timeout clock for this retry, stay in ST_PICKUP
        } else {
          setState(ST_SEARCH, "pickup finished - resuming search");
        }
      }
      break;
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= TELEMETRY_MS) {
    lastPrint = now;
    Serial.print(now);        Serial.print('\t'); Serial.print(stateName(state)); Serial.print('\t');
    Serial.print(notch.mm);   Serial.print('\t'); Serial.print(top.mm);           Serial.print('\t');
    Serial.print(searchL.mm); Serial.print('\t'); Serial.print(searchR.mm);       Serial.print('\t');
    Serial.print(metal);      Serial.print('\t');
    Serial.print(diff);       Serial.print('\t'); Serial.print(mismatchConfirmed); Serial.print('\t');
    Serial.print(lastDriveLeftPct()); Serial.print('\t'); Serial.println(lastDriveRightPct());
  }
}
