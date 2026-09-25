#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

// ============================================================================
//  tof.cpp  -  4x ToF on the SX1509 XSHUT chain.
//
//        [BL]  notch  [BR]        bottom pair, VL53L0X - weight detection
//           [UPRIGHT]             across the notch - funnel presence
//             [REAR]              VL53L1X - reversing clearance
//
//  Init: all XSHUT low, raise one at a time, wait TOF_BOOT_MS, init,
//  re-address from TOF_ADDRESS_START (0x34) in index order, start continuous.
//
//  XSHUT is driven by writing the SX1509's bank-A registers directly (same as
//  wire_finder.cpp), so the SparkFun SX1509 library isn't needed.
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

uint16_t &tofBL      = tofMM[TOF_BL];
uint16_t &tofBR      = tofMM[TOF_BR];
uint16_t &tofUpright = tofMM[TOF_UPRIGHT];
uint16_t &tofRear    = tofMM[TOF_REAR];

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

    bool ok = false;
    if (TOF_TYPE[i] == 0)                            // VL53L0X
    {
      l0x[i].setBus(&TOF_WIRE);
      l0x[i].setTimeout(100);
      ok = l0x[i].init();
      if (ok)
      {
        l0x[i].setAddress(TOF_ADDRESS_START + i);
        l0x[i].startContinuous(TOF_PERIOD_MS);
      }
    }
    else                                             // VL53L1X
    {
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

    tofSensorOk[i] = ok;
    Serial.print("ToF "); Serial.print(i);
    Serial.print(" (XSHUT"); Serial.print(XSHUT_TOF[i]);
    Serial.print(" CON"); Serial.print(27 + XSHUT_TOF[i]); Serial.print(", ");
    Serial.print(TOF_TYPE[i] ? "L1X" : "L0X"); Serial.print(") ");
    Serial.println(ok ? "ok" : "FAILED - wrong model in TOF_TYPE, or wrong TOF_WIRE?");
  }
}

void tofUpdate()
{
  for (int i = 0; i < TOF_COUNT; i++)
  {
    if (!tofSensorOk[i]) continue;

    if (TOF_TYPE[i] == 1)
    {
      if (!l1x[i].dataReady()) continue;
      uint16_t mm = l1x[i].read(false);
      // seen 2026-09-25: "valid" status with 64351mm - reject anything past range
      bool valid = l1x[i].ranging_data.range_status == VL53L1X::RangeValid && mm <= TOF_MAX_VALID_MM;
      tofMM[i] = valid ? mm : 0;
    }
    else
    {
      if ((l0x[i].readReg(L0X_REG_INTERRUPT_STATUS) & 0x07) == 0) continue;
      uint16_t mm = l0x[i].readRangeContinuousMillimeters();
      tofMM[i] = (mm >= L0X_NO_TARGET_MM) ? 0 : mm;
    }
  }
}

bool tofOk(int index)
{
  return index >= 0 && index < TOF_COUNT && tofSensorOk[index];
}

bool rearBlocked() { return tofRear > 0 && tofRear < REAR_STOP_MM; }
