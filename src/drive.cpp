#include "drive.h"
#include "motor.h"
#include "config.h"

static int lastLeftPct  = 0;
static int lastRightPct = 0;

static inline int clampPct(int v) { return v > 100 ? 100 : (v < -100 ? -100 : v); }

// motor.h takes a magnitude (0-100) + explicit forward/backward call, so a
// signed percent needs routing to the right function.
static void driveOneMotor(int pct, int motorNum)
{
  pct = clampPct(pct);
  if (pct > 0)      motorForward(pct, motorNum);
  else if (pct < 0) motorBackward(-pct, motorNum);
  else              motorStop(motorNum);
}

void drive(int leftPct, int rightPct)
{
  lastLeftPct  = clampPct(leftPct);
  lastRightPct = clampPct(rightPct);
  driveOneMotor(lastLeftPct,  1);   // motor 1 = LEFT
  driveOneMotor(lastRightPct, 2);   // motor 2 = RIGHT
}

void driveForward() { drive(+CRUISE_SPEED_PCT, +CRUISE_SPEED_PCT); }
void driveReverse() { drive(-CRUISE_SPEED_PCT, -CRUISE_SPEED_PCT); }
void turnRight()    { drive(+TURN_SPEED_PCT,  -TURN_SPEED_PCT);  }
void turnLeft()     { drive(-TURN_SPEED_PCT,  +TURN_SPEED_PCT);  }
void stopMotors()   { drive(0, 0); }

bool isReversing() { return lastLeftPct < 0 && lastRightPct < 0; }
