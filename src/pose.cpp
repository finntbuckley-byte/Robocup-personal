#include "pose.h"
#include <Arduino.h>
#include <math.h>
#include "odometry.h"
#include "imu.h"

// ============================================================================
//  pose.cpp  -  see pose.h. Pure bookkeeping: reads odometry + IMU, drives
//  nothing.
//
//  Uses the MEAN of the previous and current heading for each step (a turn
//  while moving would otherwise put the whole step on the new heading).
//  Distance is signed, so reversing moves the pose backwards correctly.
// ============================================================================

static float px = 0, py = 0;
static float lastDist = 0;
static float lastHeading = 0;

static float wrap180(float d)
{
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void poseReset()
{
  px = py = 0;
  lastDist = odomDistanceMM();
  lastHeading = imuHeadingDeg();
}

void poseUpdate()
{
  float dist = odomDistanceMM();
  float step = dist - lastDist;
  lastDist = dist;

  float heading = imuHeadingDeg();
  // mean heading across the step, taking the short way round +/-180
  float mid = lastHeading + wrap180(heading - lastHeading) * 0.5f;
  lastHeading = heading;

  if (step == 0.0f) return;
  float rad = mid * (float)PI / 180.0f;
  px += step * cosf(rad);
  py += step * sinf(rad);
}

float poseXmm() { return px; }
float poseYmm() { return py; }
float poseDistHomeMM() { return sqrtf(px * px + py * py); }
float poseBearingHomeDeg() { return atan2f(-py, -px) * 180.0f / (float)PI; }
