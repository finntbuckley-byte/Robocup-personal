#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "imu.h"
#include "config.h"

static Adafruit_BNO055 bno(55, IMU_ADDR, &IMU_WIRE);
static bool  ok = false;
static float rawDeg = 0;      // BNO heading, 0-360
static float zeroDeg = 0;
static float rateDps = 0;
static uint8_t gyroCal = 0;
static uint32_t lastValidAt = 0;
static bool dataValid = false;

static float wrap180(float d)
{
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// ---- brownout detection + recovery ------------------------------------------
// A BNO055 that browns out reboots into CONFIG mode (OPR_MODE 0x00): its
// fused heading then stops updating, so the robot would steer on a frozen
// number while imuOk() still says true. Seen 2026-09-28 (hdg 0.0 all run).
// Every read checks OPR_MODE; if it isn't IMUPLUS the mode is written back
// (non-blocking, the chip needs ~7ms to switch) and the heading is re-zeroed
// so it carries on from its last good value instead of jumping.
static const uint8_t BNO_REG_OPR_MODE = 0x3D;
static const uint8_t BNO_MODE_IMUPLUS = 0x08;
static uint8_t oprMode = 0xFF;
static int     resetCount = 0;
static float   lastGoodHeading = 0;
static unsigned long recoverUntil = 0;    // heading frozen at lastGoodHeading until then

static bool readReg(uint8_t reg, uint8_t &val)
{
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(reg);
  if (IMU_WIRE.endTransmission(false) != 0) return false;
  if (IMU_WIRE.requestFrom(IMU_ADDR, (uint8_t)1) != 1) return false;
  val = IMU_WIRE.read();
  return true;
}

static void writeReg(uint8_t reg, uint8_t val)
{
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(val);
  IMU_WIRE.endTransmission();
}

static float wrap180(float d);

// true = the chip is in IMUPLUS and its data can be trusted this read
static bool checkMode()
{
  if (!readReg(BNO_REG_OPR_MODE, oprMode)) { oprMode = 0xFE; return false; }
  if (oprMode == BNO_MODE_IMUPLUS) return millis() >= recoverUntil;

  resetCount++;
  Serial.print("!! IMU RESET #"); Serial.print(resetCount);
  Serial.print(" (OPR_MODE 0x"); Serial.print(oprMode, HEX);
  Serial.print(") - back to IMUPLUS, heading held at "); Serial.println(lastGoodHeading, 1);
  writeReg(BNO_REG_OPR_MODE, BNO_MODE_IMUPLUS);
  recoverUntil = millis() + IMU_RECOVER_MS;
  return false;
}

static void read()
{
  static bool rezero = false;
  const int resetsBefore=resetCount;
  const bool modeValid=checkMode();
  // A failed I2C read is not proof of a reset. Re-zero only after an actual
  // non-IMUPLUS mode was observed, otherwise rotation during a dropout is lost.
  if(resetCount!=resetsBefore) rezero=true;
  if (!modeValid) {
      // A single failed transfer is not a reset. The last good sample remains
      // usable only within imuOk()'s bounded freshness window.
      if(oprMode != 0xFE) dataValid = false;
      return;
  }

  // Explicit transfer checks; the library getEvent API does not report a
  // failed vector read reliably enough to authorize autonomous homing.
  // 0x18/19 = gyro Z; 0x1A/1B = Euler heading. The following registers
  // are roll and pitch, NOT heading. Keep the burst limited to these four.
  uint8_t values[4];
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(Adafruit_BNO055::BNO055_GYRO_DATA_Z_LSB_ADDR);
  if(IMU_WIRE.endTransmission(false)!=0 || IMU_WIRE.requestFrom(IMU_ADDR,uint8_t(4))!=4) {
      return;
  }
  for(auto &v:values) v=IMU_WIRE.read();
  const float heading = uint16_t(values[2] | (values[3]<<8)) / 16.0f;
  if(heading>=360) return; // retain the last valid sample, never a corrupt angle
  rawDeg = heading;
  rateDps = IMU_GYRO_SIGN * int16_t(values[0] | (values[1]<<8)) / 16.0f;
  dataValid=true; lastValidAt=millis();
  uint8_t s, a, m;
  bno.getCalibration(&s, &gyroCal, &a, &m);

  // after a reset the fused heading restarts from 0: shift the zero so the
  // reported heading continues from the last good value
  if (rezero)
  {
    zeroDeg = rawDeg - IMU_HEADING_SIGN * lastGoodHeading;
    rezero = false;
  }
  lastGoodHeading = wrap180(IMU_HEADING_SIGN * (rawDeg - zeroDeg));
}

uint8_t imuOprMode()  { return oprMode; }
int     imuResetCount() { return resetCount; }

bool imuInit()
{
  uint8_t chip=0;
  if (!readReg(0x00,chip) || chip!=0xA0) { ok = false; return false; }
  ok = bno.begin(OPERATION_MODE_IMUPLUS);
  if (ok) { delay(50); read(); zeroDeg = rawDeg; }
  return ok;
}

void imuUpdate()
{
  static unsigned long last = 0;
  if (!ok || millis() - last < IMU_READ_MS) return;
  last = millis();
  read();
}

void imuZero()          { if (ok) { read(); if(imuOk()) {zeroDeg = rawDeg;lastGoodHeading=0;} } }
bool imuOk()            { return ok && dataValid && millis()-lastValidAt<150; }
float imuHeadingDeg()   { return ok ? wrap180(IMU_HEADING_SIGN * (rawDeg - zeroDeg)) : 0.0f; }
float imuRateDps()      { return ok ? rateDps : 0.0f; }
uint8_t imuGyroCal()    { return gyroCal; }

static HeadingTuning tuning = { HEADING_KP, HEADING_KI, HEADING_KD,
                                 HEADING_I_MAX, HEADING_I_START, HEADING_MAX_STEER };
HeadingTuning &headingTuning() { return tuning; }

static float integ = HEADING_I_START; // % of steer from the I term
static unsigned long lastHoldMs = 0;

void headingHoldReset() { integ = tuning.iStart; lastHoldMs = 0; }
float headingHoldIntegral() { return integ; }

// PID on heading error, the D from the gyro rate (no noisy differentiation).
// err > 0 = we're LEFT of target -> steer right.
int headingHoldSteer(float targetDeg)
{
  if (!ok) return 0;
  float err = wrap180(targetDeg - imuHeadingDeg());

  unsigned long now = millis();
  if (lastHoldMs != 0 && now - lastHoldMs <= 100)
  {
    integ += tuning.ki * err * (now - lastHoldMs) / 1000.0f;
    if (integ >  tuning.iMax) integ =  tuning.iMax;
    if (integ < -tuning.iMax) integ = -tuning.iMax;
  }
  lastHoldMs = now;

  float s = tuning.kp * err + integ - tuning.kd * rateDps;
  if (s >  tuning.maxSteer) s =  tuning.maxSteer;
  if (s < -tuning.maxSteer) s = -tuning.maxSteer;
  return (int)(s >= 0 ? s + 0.5f : s - 0.5f);
}
