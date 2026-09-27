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
  if (!readReg(BNO_REG_OPR_MODE, oprMode)) { oprMode = 0xFE; return false; }   // no answer (mid-reboot)
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
  if (!checkMode()) { rezero = true; return; }   // keep last values while it's rebooting / switching

  sensors_event_t e;
  bno.getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
  rawDeg = e.orientation.x;
  bno.getEvent(&e, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rateDps = IMU_GYRO_SIGN * e.gyro.z * 57.2958f;   // + = turning right
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
  IMU_WIRE.beginTransmission(IMU_ADDR);
  if (IMU_WIRE.endTransmission() != 0) { ok = false; return false; }
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

void imuZero()          { if (ok) { read(); zeroDeg = rawDeg; } }
bool imuOk()            { return ok; }
float imuHeadingDeg()   { return ok ? wrap180(IMU_HEADING_SIGN * (rawDeg - zeroDeg)) : 0.0f; }
float imuRateDps()      { return ok ? rateDps : 0.0f; }
uint8_t imuGyroCal()    { return gyroCal; }

static float integ = HEADING_I_START; // % of steer from the I term
static unsigned long lastHoldMs = 0;

void headingHoldReset() { integ = HEADING_I_START; lastHoldMs = 0; }
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
    integ += HEADING_KI * err * (now - lastHoldMs) / 1000.0f;
    if (integ >  HEADING_I_MAX) integ =  HEADING_I_MAX;
    if (integ < -HEADING_I_MAX) integ = -HEADING_I_MAX;
  }
  lastHoldMs = now;

  float s = HEADING_KP * err + integ - HEADING_KD * rateDps;
  if (s >  HEADING_MAX_STEER) s =  HEADING_MAX_STEER;
  if (s < -HEADING_MAX_STEER) s = -HEADING_MAX_STEER;
  return (int)(s >= 0 ? s + 0.5f : s - 0.5f);
}
