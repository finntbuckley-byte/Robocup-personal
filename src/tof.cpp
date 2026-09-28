#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <stdio.h>   // snprintf, for the live ToF line's fixed-width padding

// ============================================================================
//  tof.cpp  -  5x ToF on the SX1509 XSHUT chain.
//
//        [BL]  notch  [BR]        bottom pair, VL53L0X - weight detection
//           [UPRIGHT]             across the notch - lying-weight "bottom"
//             [REAR]              VL53L1X - reversing clearance
//             [TOP]               VL53L0X - lying-weight "top" (proposed,
//                                  not yet fitted - see config.h TOF_TOP)
//
//  Init: all XSHUT low, raise one at a time, wait TOF_BOOT_MS, init,
//  re-address from TOF_ADDRESS_START (0x34) in index order, start continuous.
//
//  XSHUT is driven by writing the SX1509's bank-A registers directly (same as
//  wire_finder.cpp), so the SparkFun SX1509 library isn't needed.
//
//  Stale + recovery (2026-09-28: the notch L1X kept dropping off the bus on
//  a bad connection at its end and rebooting to 0x29 - its last reading then
//  stayed frozen in tofMM): a sensor with no new data for TOF_STALE_MS reads
//  0 and tofOk() goes false; every TOF_RECOVER_INTERVAL_MS (backing off to
//  TOF_RECOVER_BACKOFF_MS after 3 failures in a row) its XSHUT is pulsed and
//  it's set up again - the same reset that brought it back after a re-flash.
//  A recovery attempt blocks ~60-150 ms, hence the rate limit.
//
//  Reads never block:
//    L1X - gated on dataReady(); invalid range status reads as 0.
//    L0X - gated on the result-interrupt register, so
//          readRangeContinuousMillimeters() returns immediately instead of
//          spinning for up to the timeout.
// ============================================================================

static VL53L0X l0x[TOF_COUNT];
static VL53L1X l1x[TOF_COUNT];

uint16_t tofMM[TOF_COUNT] = {0};
static bool tofSensorOk[TOF_COUNT] = {false};

// diagnostics ('u' in nav): the last raw reading, even when rejected
static uint16_t rawMM[TOF_COUNT] = {0};
static uint8_t  rawStatus[TOF_COUNT] = {0};     // L1X range_status, 0 = valid
static unsigned long lastDataMs[TOF_COUNT] = {0};

// stale detection + recovery
static bool tofFresh[TOF_COUNT] = {false};
static unsigned long setupAtMs[TOF_COUNT] = {0};
static unsigned long lastRecoverMs[TOF_COUNT] = {0};
static uint8_t recoverFailsInRow[TOF_COUNT] = {0};
static int recoverCount = 0;
static bool recoveryPending[TOF_COUNT] = {false};   // re-initialised, waiting for its first reading

uint16_t &tofBL      = tofMM[TOF_BL];
uint16_t &tofBR      = tofMM[TOF_BR];
uint16_t &tofUpright = tofMM[TOF_UPRIGHT];
uint16_t &tofRear    = tofMM[TOF_REAR];
uint16_t &tofTop     = tofMM[TOF_TOP];

static uint8_t xshutMask = 0;

// VL53L0X RESULT_INTERRUPT_STATUS - low 3 bits non-zero = new range ready
static const uint8_t L0X_REG_INTERRUPT_STATUS = 0x13;
static const uint16_t L0X_NO_TARGET_MM = 8000;   // 8190/8191 = out of range

static inline bool tofIndexEnabled(int i)
{
#if !USE_REAR_TOF
  if (i == TOF_REAR) return false;
#endif
  return XSHUT_TOF[i] >= 0;
}

static bool sxWrite(uint8_t reg, uint8_t val)
{
  SX_WIRE.beginTransmission(SX1509_ADDRESS);
  SX_WIRE.write(reg);
  SX_WIRE.write(val);
  return SX_WIRE.endTransmission() == 0;
}

static void xshutWrite(uint8_t mask)
{
  xshutMask = mask;
  sxWrite(SX_REG_DATA_A, xshutMask);
}

// set up sensor i, which must be the only one answering on 0x29 (its XSHUT
// just raised). Fresh driver object = default address. Used at boot and by
// the recovery.
static bool setupSensor(int i)
{
  bool ok = false;
  if (TOF_TYPE[i] == 0)                              // VL53L0X
  {
    l0x[i] = VL53L0X();
    l0x[i].setBus(&TOF_WIRE);
    l0x[i].setTimeout(100);
    ok = l0x[i].init();
    if (ok)
    {
      l0x[i].setAddress(TOF_ADDRESS_START + i);
      l0x[i].startContinuous(TOF_PERIOD_MS);
    }
  }
  else                                               // VL53L1X
  {
    l1x[i] = VL53L1X();
    l1x[i].setBus(&TOF_WIRE);
    l1x[i].setTimeout(100);
    ok = l1x[i].init();
    if (ok)
    {
      l1x[i].setAddress(TOF_ADDRESS_START + i);
      l1x[i].setDistanceMode(TOF_SHORT_MODE[i] ? VL53L1X::Short : VL53L1X::Long);
      l1x[i].setMeasurementTimingBudget(TOF_L1X_BUDGET_US);
      l1x[i].startContinuous(TOF_PERIOD_MS);
    }
  }
  setupAtMs[i] = millis();
  lastDataMs[i] = 0;
  tofFresh[i] = ok;
  return ok;
}

// pulse sensor i's XSHUT (clean reset, whatever state it's in), set it up again.
// 0x29 must be free: every OTHER stale sensor is held in shutdown first (one
// that rebooted by itself would be squatting on 0x29), and a sensor whose
// recovery fails is left in shutdown for the same reason. (First version, 28/9:
// a failed L0X stayed up at 0x29, so the other L0X's recovery collided with it
// and both failed forever.)
static void recoverSensor(int i)
{
  lastRecoverMs[i] = millis();
  uint8_t mask = xshutMask;
  for (int j = 0; j < TOF_COUNT; j++)
    if (j != i && tofSensorOk[j] && !tofFresh[j]) mask &= ~(1 << XSHUT_TOF[j]);
  mask &= ~(1 << XSHUT_TOF[i]);
  xshutWrite(mask);
  delay(TOF_RESET_LOW_MS);
  xshutWrite(mask | (1 << XSHUT_TOF[i]));
  delay(TOF_BOOT_MS);
  bool ok = setupSensor(i);
  // a recovery only COUNTS once real data arrives (markData) - a sensor with
  // a bad connection re-inits fine and then dies the moment it ranges, and
  // must still back off (first version 28/9 retried every 2s forever)
  if (ok) { recoveryPending[i] = true; tofFresh[i] = false; }
  else
  {
    xshutWrite(xshutMask & ~(1 << XSHUT_TOF[i]));   // keep it off 0x29 until its next try
    if (recoverFailsInRow[i] < 255) recoverFailsInRow[i]++;
  }
  Serial.print(">>> ToF "); Serial.print(i);
  Serial.println(ok ? " dropped out - re-initialised, waiting for data"
                    : " dropped out - recovery FAILED, will retry");
}

// fresh data from sensor i: completes a pending recovery
static void markData(int i)
{
  lastDataMs[i] = millis();
  tofFresh[i] = true;
  if (recoveryPending[i])
  {
    recoveryPending[i] = false;
    recoverFailsInRow[i] = 0;
    recoverCount++;
    Serial.print(">>> ToF "); Serial.print(i); Serial.println(" RECOVERED (data flowing again)");
  }
}

void tofInit()
{
  // data low first, then make bank A outputs - no glitch high on any XSHUT
  bool sxOk = sxWrite(SX_REG_DATA_A, 0x00) && sxWrite(SX_REG_DIR_A, 0x00);
  if (!sxOk)
  {
    Serial.println("!! SX1509 (0x3F) not found - check SX_WIRE in config.h and"
                   " the CON26 I2C In cable. ToFs disabled.");
    return;
  }
  xshutMask = 0;
  delay(10);

  for (int i = 0; i < TOF_COUNT; i++)
  {
    if (!tofIndexEnabled(i))
    {
      Serial.print("ToF "); Serial.print(i); Serial.println(" disabled - skipped");
      continue;
    }

    xshutWrite(xshutMask | (1 << XSHUT_TOF[i]));
    delay(TOF_BOOT_MS);

    bool ok = setupSensor(i);
    tofSensorOk[i] = ok;
    // failed: hold it in shutdown so it can't sit on 0x29 and block the
    // next sensor's setup; tofUpdate() keeps retrying it (loose cable reseated)
    if (!ok) xshutWrite(xshutMask & ~(1 << XSHUT_TOF[i]));
    Serial.print("ToF "); Serial.print(i);
    Serial.print(" (XSHUT"); Serial.print(XSHUT_TOF[i]);
    Serial.print(" CON"); Serial.print(27 + XSHUT_TOF[i]); Serial.print(", ");
    Serial.print(TOF_TYPE[i] ? "L1X" : "L0X"); Serial.print(") ");
    Serial.println(ok ? "ok" : "FAILED - wrong model in TOF_TYPE, or wrong TOF_WIRE?");
  }
}

void tofUpdate()
{
  unsigned long now = millis();
  // the stale clock starts at the first update, not at setup: boot blocks
  // (the 8x8's 5s mode set) must not count as "no data" (first version 28/9
  // marked every sensor stale straight after boot)
  static unsigned long firstUpdateMs = 0;
  if (firstUpdateMs == 0) firstUpdateMs = now;
  for (int i = 0; i < TOF_COUNT; i++)
  {
    if (!tofSensorOk[i])
    {
      // failed at boot (28/9: CON28 cable knocked loose) - keep trying it
      if (!tofIndexEnabled(i)) continue;
      unsigned long wait = recoverFailsInRow[i] >= 3 ? TOF_RECOVER_BACKOFF_MS : TOF_RECOVER_INTERVAL_MS;
      if (now - lastRecoverMs[i] >= wait)
      {
        recoverSensor(i);
        if (recoveryPending[i]) tofSensorOk[i] = true;   // set up now - normal stale/recovery rules apply
      }
      continue;
    }

    // stale: no new data for TOF_STALE_MS since the last reading (or since setup)
    unsigned long since = lastDataMs[i] ? lastDataMs[i]
                                        : (setupAtMs[i] > firstUpdateMs ? setupAtMs[i] : firstUpdateMs);
    if (now - since > TOF_STALE_MS)
    {
      if (recoveryPending[i])
      {
        recoveryPending[i] = false;
        if (recoverFailsInRow[i] < 255) recoverFailsInRow[i]++;
      }
      if (tofFresh[i])
      {
        tofFresh[i] = false;
        tofMM[i] = 0;                 // never act on a frozen old value
        Serial.print(">>> ToF "); Serial.print(i); Serial.println(" STALE - no data, trying recovery");
      }
      unsigned long wait = recoverFailsInRow[i] >= 3 ? TOF_RECOVER_BACKOFF_MS : TOF_RECOVER_INTERVAL_MS;
      if (now - lastRecoverMs[i] >= wait) recoverSensor(i);
      continue;
    }

    if (TOF_TYPE[i] == 1)
    {
      if (!l1x[i].dataReady()) continue;
      uint16_t mm = l1x[i].read(false);
      rawMM[i] = mm; rawStatus[i] = l1x[i].ranging_data.range_status; markData(i);
      // seen 2026-09-25: "valid" status with 64351mm - reject anything past range
      bool valid = l1x[i].ranging_data.range_status == VL53L1X::RangeValid && mm <= TOF_MAX_VALID_MM;
      tofMM[i] = valid ? mm : 0;
    }
    else
    {
      if ((l0x[i].readReg(L0X_REG_INTERRUPT_STATUS) & 0x07) == 0) continue;
      uint16_t mm = l0x[i].readRangeContinuousMillimeters();
      rawMM[i] = mm; rawStatus[i] = 0; markData(i);
      tofMM[i] = (mm >= L0X_NO_TARGET_MM) ? 0 : mm;
    }
  }
}

bool tofOk(int index)
{
  return index >= 0 && index < TOF_COUNT && tofSensorOk[index] && tofFresh[index];
}

int tofRecoverCount() { return recoverCount; }

bool rearBlocked() { return tofRear > 0 && tofRear < REAR_STOP_MM; }

// matched against the TOF_* index constants (config.h), not a fixed order,
// so this stays correct if the index assignment ever changes there.
static const char* tofName(int i)
{
  if (i == TOF_BL)      return "BL";
  if (i == TOF_BR)      return "BR";
  if (i == TOF_UPRIGHT) return "UPRIGHT";
  if (i == TOF_REAR)    return "REAR";
  if (i == TOF_TOP)     return "TOP";
  return "?";
}

// ---------------------------------------------------------------------------
// LIVE ToF LINE  -  'u' toggles this on/off (nav_main.cpp). A header row
// (sensor names) is printed once, then a single line is overwritten in
// place ('\r', no newline) every RAW_LIVE_MS instead of scrolling a new
// block of rows on every press - much easier to watch while testing.
// Each cell is padded to a fixed width so a shorter new value fully
// overwrites a longer old one (no leftover characters from the previous
// print). ok/status/age/i2c/addr detail from the old one-shot dump isn't
// shown here - it's still in the main 't' telemetry (TOP/TOPok columns) for
// anything needing the full picture.
// ---------------------------------------------------------------------------
static bool rawLiveOn = false;
static unsigned long rawLiveLast = 0;
static const unsigned long RAW_LIVE_MS = 150;

static void printRawHeader()
{
  Serial.println();
  char buf[16];
  for (int i = 0; i < TOF_COUNT; i++)
  {
    snprintf(buf, sizeof(buf), "%-12s", tofName(i));
    Serial.print(buf);
  }
  Serial.println();
}

static void printRawLive()
{
  Serial.print('\r');
  char cell[16], buf[16];
  for (int i = 0; i < TOF_COUNT; i++)
  {
    snprintf(cell, sizeof(cell), "%s%4u", tofSensorOk[i] ? "ok " : "-- ", tofMM[i]);
    snprintf(buf, sizeof(buf), "%-12s", cell);
    Serial.print(buf);
  }
}

// 'u': toggle the live line on/off
void tofPrintRaw()
{
  rawLiveOn = !rawLiveOn;
  if (rawLiveOn) { printRawHeader(); rawLiveLast = 0; }
  else            Serial.println("\nToF live stopped");
}

// called every loop() (nav_main.cpp) - no-op unless the live line is on
void tofPrintRawTick()
{
  if (!rawLiveOn) return;
  if (millis() - rawLiveLast < RAW_LIVE_MS) return;
  rawLiveLast = millis();
  printRawLive();
}
