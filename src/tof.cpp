#include "tof.h"
/*#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

// ============================================================================
//  tof.cpp  -  7x ToF: low pair (short range, weight) + top-front pair +
//  top-corner pair (long range, obstacle) + rear (long range, obstacle).
//
//        [CL] 45out          45out [CR]     angled corner cover
//        [FL]      [FR]                     top-front tier - obstacle
//        [BL]      [BR]                     low tier - weight detection
//              [REAR]                       reversing clearance
//
//  Init: all XSHUT low via the SX1509, raise one at a time, delay(50), init,
//  re-address from 0x30 in index order, startContinuous(50), no distance-
//  mode/timing-budget calls.
//
//  Reads: L1X gated on dataReady() (non-blocking); L0X on a fixed schedule
//  so continuous data is already waiting when we ask.
// ============================================================================

SX1509 io;

static VL53L0X l0x[TOF_COUNT];
static VL53L1X l1x[TOF_COUNT];

uint16_t tofMM[TOF_COUNT] = {0};
static bool tofSensorOk[TOF_COUNT] = {false};

uint16_t &tofBL   = tofMM[TOF_BL];
uint16_t &tofBR   = tofMM[TOF_BR];
uint16_t &tofFL   = tofMM[TOF_FL];
uint16_t &tofFR   = tofMM[TOF_FR];
uint16_t &cornerL = tofMM[TOF_CL];
uint16_t &cornerR = tofMM[TOF_CR];
uint16_t &tofRear = tofMM[TOF_REAR];

static unsigned long tofLastRead = 0;

static inline bool tofIndexEnabled(int i)
{
#if !USE_CORNER_TOF
  if (i == TOF_CL || i == TOF_CR) return false;
#endif
#if !USE_REAR_TOF
  if (i == TOF_REAR) return false;
#endif
  return XSHUT_TOF[i] >= 0;
}

static inline uint16_t normL0X(uint16_t mm) { return (mm >= 8000) ? 0 : mm; }
static inline uint16_t normL1X(uint16_t mm) { return mm; }   // 0 on no target

void tofInit()
{
  if (!io.begin(SX1509_ADDRESS))
    Serial.println("SX1509 (0x3F) not found - check its 4-pin I2C cable");

  for (int i = 0; i < TOF_COUNT; i++)
  {
    if (XSHUT_TOF[i] < 0) continue;
    io.pinMode(XSHUT_TOF[i], OUTPUT);
    io.digitalWrite(XSHUT_TOF[i], LOW);
  }
  delay(10);

  for (int i = 0; i < TOF_COUNT; i++)
  {
    if (!tofIndexEnabled(i))
    {
      Serial.print("ToF "); Serial.print(i); Serial.println(" disabled/unset - skipped");
      continue;
    }

    io.digitalWrite(XSHUT_TOF[i], HIGH);
    delay(50);                                       // proven necessary

    bool ok = false;
    if (TOF_TYPE[i] == 0)                            // VL53L0X
    {
      l0x[i].setTimeout(100);
      ok = l0x[i].init();
      if (ok) { l0x[i].setAddress(TOF_ADDRESS_START + i); l0x[i].startContinuous(50); }
    }
    else                                             // VL53L1X
    {
      l1x[i].setTimeout(100);
      ok = l1x[i].init();
      if (ok) { l1x[i].setAddress(TOF_ADDRESS_START + i); l1x[i].startContinuous(50); }
    }

    tofSensorOk[i] = ok;
    Serial.print("ToF "); Serial.print(i);
    Serial.print(" (XSHUT"); Serial.print(XSHUT_TOF[i]); Serial.print(", ");
    Serial.print(TOF_TYPE[i] ? "L1X" : "L0X"); Serial.print(") ");
    Serial.println(ok ? "ok" : "FAILED");
  }
}

void tofUpdate()
{
  for (int i = 0; i < TOF_COUNT; i++)
    if (tofSensorOk[i] && TOF_TYPE[i] == 1 && l1x[i].dataReady())
      tofMM[i] = normL1X(l1x[i].read(false));

  if (millis() - tofLastRead < TOF_READ_MS) return;
  tofLastRead = millis();

  for (int i = 0; i < TOF_COUNT; i++)
    if (tofSensorOk[i] && TOF_TYPE[i] == 0)
      tofMM[i] = normL0X(l0x[i].readRangeContinuousMillimeters());
}

// ---- interpretation helpers ----

uint16_t frontWallMM()
{
  uint16_t l = tofFL, r = tofFR;
  if (l == 0) return r;
  if (r == 0) return l;
  return (l < r) ? l : r;
}

static inline bool pairSeesWeight(uint16_t bot, uint16_t top)
{
  if (bot < WEIGHT_MIN_MM || bot == 0 || bot > WEIGHT_MAX_MM) return false;
  if (top == 0) return true;
  return top >= bot + DIFF_CLEAR_MARGIN_MM;
}

bool weightLeft()  { return pairSeesWeight(tofBL, tofFL); }
bool weightRight() { return pairSeesWeight(tofBR, tofFR); }

bool cornerNearLeft()  { return cornerL > 0 && cornerL < CORNER_NEAR_MM; }
bool cornerNearRight() { return cornerR > 0 && cornerR < CORNER_NEAR_MM; }
bool rearBlocked()     { return tofRear > 0 && tofRear < REAR_STOP_MM; }
*/