// stack_test.cpp - creep on GO until notch+top AGREE (small, consistent
// |notch_mm - top_mm|) AND inductive reads metal, then stop and run the
// crane. Bench data (28/9): upright real/dummy read both sensors in the
// SAME ~60-110mm band, but that band overlaps a LYING weight's readings
// (30-90 or 50-130 depending on which way it fell) too much to tell them
// apart on absolute distance alone - the gap between the two sensors is
// the actual signal (small+steady = upright, bigger = lying), so that's
// what gates the pickup here. A confirmed, persistent MISMATCH aborts the
// attempt outright rather than waiting for the creep timeout.
// 115200 baud, TSV output.
// Serial: x = stop now | r = re-arm | s = print bands
//         a<mm> notch min | b<mm> notch max | c<mm> top min | d<mm> top max
//         e<mm> max |notch-top| to count as agreement | f<ms> confirm time
// Standalone test module: does NOT touch tof.cpp / navigation.cpp.
#include <Arduino.h>
#include <Wire.h>
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

struct Tof {
  VL53L1X s;
  bool ok = false;
  uint16_t mm = 0, raw = 0;
  uint8_t st = 255;
  unsigned long lastMs = 0;
};
static Tof notch, top;

enum State { ST_WAIT, ST_CREEP, ST_SETTLE, ST_PICKUP, ST_DONE };
static State state = ST_WAIT;
static unsigned long stateMs = 0;

// runtime-tunable bands (start from config.h)
static int nLo = STACK_NOTCH_MIN_MM, nHi = STACK_NOTCH_MAX_MM;
static int tLo = STACK_TOP_MIN_MM,   tHi = STACK_TOP_MAX_MM;
static int dMax = STACK_DISCREPANCY_MM;
static unsigned long matchMs = STACK_MATCH_CONFIRM_MS;

static void setState(State s, const char *why) {
  state = s; stateMs = millis();
  Serial.print(">>> state "); Serial.print((int)s); Serial.print(' '); Serial.println(why);
}

// ---- SX1509 XSHUT + sensor bring-up (same approach as tof.cpp) -------------
static bool sxWrite(uint8_t reg, uint8_t val) {
  SX_WIRE.beginTransmission(SX1509_ADDRESS);
  SX_WIRE.write(reg); SX_WIRE.write(val);
  return SX_WIRE.endTransmission() == 0;
}

static bool bringUp(Tof &t, int xshut, uint8_t addr, uint8_t &mask, int roi) {
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

static void tofRead(Tof &t, unsigned long now) {
  if (!t.ok) return;
  if (t.s.dataReady()) {
    t.raw = t.s.read(false);
    t.st  = t.s.ranging_data.range_status;
    t.lastMs = now;
    bool valid = (t.st == VL53L1X::RangeValid) && t.raw <= TOF_MAX_VALID_MM;
    t.mm = valid ? t.raw : 0;
  }
  if (!t.lastMs || now - t.lastMs > TOF_STALE_MS) t.mm = 0;   // never act on a frozen value
}
static bool fresh(const Tof &t, unsigned long now) {
  return t.ok && t.lastMs && now - t.lastMs <= TOF_STALE_MS;
}

// ---- inductive, debounced (metal must read steadily) -----------------------
static bool inductiveMetal(unsigned long now) {
  static unsigned long since = 0;
  bool raw = (digitalRead(PIN_INDUCTIVE) == (INDUCTIVE_ACTIVE_LOW ? LOW : HIGH));
  if (!raw) { since = 0; return false; }
  if (!since) since = now;
  return now - since >= INDUCTIVE_CONFIRM_MS;
}

static bool inBand(uint16_t mm, int lo, int hi) { return lo > 0 && mm >= lo && mm <= hi; }

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

static void printBands() {
  Serial.print("bands notch "); Serial.print(nLo); Serial.print('-'); Serial.print(nHi);
  Serial.print("  top "); Serial.print(tLo); Serial.print('-'); Serial.println(tHi);
  if (nLo <= 0 || tLo <= 0) Serial.println("!! bands unset - pickup can't fire; set with a/b/c/d");
  Serial.print("discrepancy max "); Serial.print(dMax); Serial.print("mm, confirm ");
  Serial.print(matchMs); Serial.println("ms (set with e/f)");
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
      case 'a': nLo = v; break;  case 'b': nHi = v; break;
      case 'c': tLo = v; break;  case 'd': tHi = v; break;
      case 'e': dMax = v; break; case 'f': matchMs = v; break;
      default: continue;
    }
    printBands();
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

  // all XSHUT low first; the bottom pair and rear ToF stay in shutdown in this test
  bool sx = sxWrite(SX_REG_DATA_A, 0x00) && sxWrite(SX_REG_DIR_A, 0x00);
  if (!sx) { Serial.println("!! SX1509 (0x3F) not found - check SX_WIRE and CON26 cable"); return; }
  uint8_t mask = 0;
  delay(10);
  bool a = bringUp(notch, STACK_XSHUT_NOTCH, TOF_ADDRESS_START + 0, mask, 16);
  bool b = bringUp(top,   STACK_XSHUT_TOP,   TOF_ADDRESS_START + 1, mask, STACK_TOP_ROI);
  Serial.println("BOOT");
  Serial.print("notch (XSHUT"); Serial.print(STACK_XSHUT_NOTCH); Serial.println(a ? ") ok" : ") FAILED");
  Serial.print("top   (XSHUT"); Serial.print(STACK_XSHUT_TOP);   Serial.println(b ? ") ok" : ") FAILED - is it an L1X?");
  printBands();
  Serial.println("ms\tstate\tnotch_mm\tnotch_raw\tnotch_st\ttop_mm\ttop_raw\ttop_st\tmetal\tnotch_in\ttop_in\tdiff\tmatch\tmismatch");
}

void loop() {
  unsigned long now = millis();
  hwUpdate();
  tofRead(notch, now);
  tofRead(top, now);
  handleSerial();

  bool metal     = inductiveMetal(now);
  bool notchIn   = inBand(notch.mm, nLo, nHi);   // informational only - see discrepancy check below
  bool topIn     = inBand(top.mm, tLo, tHi);
  bool bothFresh = fresh(notch, now) && fresh(top, now);

  // Both sensors must have an actual valid return (not "lost target", which
  // reads mm=0) before a small gap between them means anything - otherwise
  // two sensors both seeing nothing would look like a perfect "agreement".
  bool bothValid = notch.mm > 0 && top.mm > 0;
  int diff = bothValid ? (int)notch.mm - (int)top.mm : 999;
  if (diff < 0) diff = -diff;
  bool agree    = bothValid && diff <= dMax;
  bool disagree = bothValid && diff >  dMax;

  // debounced BOTH ways - one noisy frame shouldn't trigger a pickup OR
  // abort one; see config.h STACK_DISCREPANCY_MM/STACK_MATCH_CONFIRM_MS
  static unsigned long agreeSince = 0, disagreeSince = 0;
  if (agree)    { if (!agreeSince) agreeSince = now; }    else agreeSince = 0;
  if (disagree) { if (!disagreeSince) disagreeSince = now; } else disagreeSince = 0;
  bool matched          = agreeSince    && now - agreeSince    >= matchMs;
  bool mismatchConfirmed = disagreeSince && now - disagreeSince >= matchMs;

  bool go = goEdge(now);

  switch (state) {
    case ST_WAIT:
    case ST_DONE:
      if (go) {
        if (!bothFresh) Serial.println("!! a ToF is stale/failed - not starting");
        else setState(ST_CREEP, "GO");
      }
      break;

    case ST_CREEP:
      if (go && GO_STOPS_ROUND) { hwStop(); setState(ST_DONE, "GO pressed again"); break; }
      if (!bothFresh)  { hwStop(); setState(ST_DONE, "ToF stale - stopped"); break; }
      if (now - stateMs > STACK_CREEP_MAX_MS) { hwStop(); setState(ST_DONE, "creep timeout"); break; }
      if (mismatchConfirmed) { hwStop(); setState(ST_DONE, "mismatch confirmed - notch/top disagree, likely lying"); break; }
      if (matched && metal) { hwSoftStop(); setState(ST_SETTLE, "notch/top agree + metal"); break; }
      hwDrive(CREEP_SPEED_PCT, CREEP_SPEED_PCT);
      break;

    case ST_SETTLE:   // soft stop ramps down; crane only starts once it has finished
      hwSoftStop();
      if (now - stateMs >= DRIVE_DECEL_MS + 100) { hwPickupStart(); setState(ST_PICKUP, "crane start"); }
      break;

    case ST_PICKUP:
      hwSoftStop();   // keeps the tracks held at zero while the crane runs
      // 100 ms guard: busy may only go true after the next collection_update()
      if ((now - stateMs > 100 && !hwPickupBusy()) || now - stateMs > PICKUP_TIMEOUT_MS) {
        Serial.println(metal ? ">>> metal STILL at notch - grab missed" : ">>> metal gone - grab ok");
        setState(ST_DONE, "pickup finished");
      }
      break;
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= TELEMETRY_MS) {
    lastPrint = now;
    Serial.print(now);      Serial.print('\t'); Serial.print((int)state);   Serial.print('\t');
    Serial.print(notch.mm); Serial.print('\t'); Serial.print(notch.raw);    Serial.print('\t');
    Serial.print(notch.st); Serial.print('\t');
    Serial.print(top.mm);   Serial.print('\t'); Serial.print(top.raw);      Serial.print('\t');
    Serial.print(top.st);   Serial.print('\t');
    Serial.print(metal);    Serial.print('\t'); Serial.print(notchIn);      Serial.print('\t');
    Serial.print(topIn);    Serial.print('\t');
    Serial.print(diff);     Serial.print('\t'); Serial.print(matched);      Serial.print('\t');
    Serial.println(mismatchConfirmed);
  }
}
