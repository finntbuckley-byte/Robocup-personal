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

static void read()
{
  sensors_event_t e;
  bno.getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
  rawDeg = e.orientation.x;
  bno.getEvent(&e, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rateDps = IMU_GYRO_SIGN * e.gyro.z * 57.2958f;   // + = turning right
  uint8_t s, a, m;
  bno.getCalibration(&s, &gyroCal, &a, &m);
}

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
