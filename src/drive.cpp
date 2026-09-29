#include "drive.h"
#include "motor.h"
#include "config.h"

static int lastLeftPct  = 0;    // last REQUESTED command (what navigation asked for)
static int lastRightPct = 0;
static float outLeft  = 0.0f;   // command actually sent, after the soft-start ramp
static float outRight = 0.0f;
static unsigned long lastDriveMs = 0;
static bool startupBoost = false;

void driveSetStartupBoost(bool enabled) { startupBoost = enabled; }

static inline int clampPct(int v) { return v > 100 ? 100 : (v < -100 ? -100 : v); }

// motor.h takes a magnitude (0-100) + explicit forward/backward call, so a
// signed percent needs routing to the right function. The magnitude is scaled
// by DRIVE_SCALE_PCT, trimmed per track and direction (DRIVE_TRIM_*), then
// motor.cpp maps 0-100% into the driver's 1.05-1.95 ms window.
static void driveOneMotor(float pct, int motorNum)
{
  const int scale = startupBoost ? STARTUP_DRIVE_SCALE_PCT : DRIVE_SCALE_PCT;
  pct = pct * scale / 100.0f;
  if (motorNum == 1) pct *= (pct >= 0 ? DRIVE_TRIM_L_FWD : DRIVE_TRIM_L_REV);   // motor 1 = LEFT
  else               pct *= (pct >= 0 ? DRIVE_TRIM_R_FWD : DRIVE_TRIM_R_REV);   // motor 2 = RIGHT
  int mag = (int)(pct >= 0 ? pct + 0.5f : -pct + 0.5f);
  if (pct > 0 && mag > 0)      motorForward(mag * MOTOR_MAX_FWD_PCT / 100, motorNum);
  else if (pct < 0 && mag > 0) motorBackward(mag * MOTOR_MAX_REV_PCT / 100, motorNum);
  else                         motorStop(motorNum);
}

// Soft start + soft stop for one track: speeding up is limited to upStep per
// call, slowing down to downStep. A direction change slows to 0 first.
static float rampToward(float out, int target, float upStep, float downStep)
{
  float mag = out < 0 ? -out : out;
  bool reversing = (target > 0 && out < 0) || (target < 0 && out > 0);
  if (reversing)
  {
    mag -= downStep;
    if (mag <= 0) return 0.0f;             // reached 0 - speed up the other way next call
    return out > 0 ? mag : -mag;
  }
  float tMag = target < 0 ? -target : target;
  if (tMag < mag)  mag = (mag - downStep > tMag) ? mag - downStep : tMag;
  else             mag = (mag + upStep   < tMag) ? mag + upStep   : tMag;
  int sign = (target != 0) ? target : (out < 0 ? -1 : 1);
  return sign < 0 ? -mag : mag;
}

void drive(int leftPct, int rightPct)
{
  lastLeftPct  = clampPct(leftPct);
  lastRightPct = clampPct(rightPct);

  // dt capped so a long gap between calls can't turn into an instant jump
  unsigned long now = millis();
  unsigned long dt  = now - lastDriveMs;
  lastDriveMs = now;
  if (dt > 50) dt = 50;
  float upStep   = 100.0f * dt / DRIVE_RAMP_MS;
  float downStep = 100.0f * dt / DRIVE_DECEL_MS;

  outLeft  = rampToward(outLeft,  lastLeftPct,  upStep, downStep);
  outRight = rampToward(outRight, lastRightPct, upStep, downStep);
  driveOneMotor(outLeft,  1);   // motor 1 = LEFT
  driveOneMotor(outRight, 2);   // motor 2 = RIGHT
}

void driveForward() { drive(+CRUISE_SPEED_PCT, +CRUISE_SPEED_PCT); }
void driveReverse() { drive(-CRUISE_SPEED_PCT, -CRUISE_SPEED_PCT); }
void turnRight()    { drive(+TURN_SPEED_PCT,  -TURN_SPEED_PCT);  }
void turnLeft()     { drive(-TURN_SPEED_PCT,  +TURN_SPEED_PCT);  }
void stopMotors()   { drive(0, 0); }   // soft stop (DRIVE_DECEL_MS) - keep calling it

// no ramp: the kill and the rear guard
void driveHardStop()
{
  lastLeftPct = lastRightPct = 0;
  outLeft = outRight = 0.0f;
  lastDriveMs = millis();
  motorStop(1);
  motorStop(2);
}

bool isReversing() { return lastLeftPct < 0 && lastRightPct < 0; }
int  lastDriveLeftPct()  { return lastLeftPct; }
int  lastDriveRightPct() { return lastRightPct; }
