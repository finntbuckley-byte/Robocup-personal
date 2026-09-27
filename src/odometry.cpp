#include <Arduino.h>
#include <Encoder.h>      // PJRC, ships with Teensyduino - interrupt-driven 4x quadrature
#include "odometry.h"
#include "config.h"
#include "drive.h"
#include "x8.h"
#include "tof.h"

static Encoder encL(PIN_ENC_L_A, PIN_ENC_L_B);
static Encoder encR(PIN_ENC_R_A, PIN_ENC_R_B);

static long  lastAvgCount = 0;     // mean of both tracks at the last update
static float distMM       = 0;     // committed, slip-corrected distance
static bool  slipping     = false;
static unsigned slipEvents = 0;

// slip-check window (only open while driving straight)
static bool  winOpen      = false;
static int   winDir       = 0;     // +1 forward (8x8), -1 reverse (rear ToF)
static unsigned long winStartMs = 0;
static float winEncMM     = 0;     // encoder travel so far in this window
static uint16_t winStartRange = 0; // 0 = nothing usable in range at the start

// stall detection
static long  stallRefL = 0, stallRefR = 0;
static unsigned long stallSinceMs = 0;
static bool  stalled      = false;

static inline float countsToMM(long c) { return c * 1000.0f / ENC_COUNTS_PER_M; }

long encLeftCount()  { return ENC_L_SIGN * encL.read(); }
long encRightCount() { return ENC_R_SIGN * encR.read(); }

static long avgCount() { return (encLeftCount() + encRightCount()) / 2; }

// range to whatever we're driving towards; 0 = nothing usable
static uint16_t rangeAhead(int dir)
{
  uint16_t r = (dir > 0) ? x8FrontMM() : tofRear;
  return (r == 0 || r > ODOM_SLIP_MAX_RANGE_MM) ? 0 : r;
}

// +1 / -1 while commanded straight forward / reverse, else 0
static int straightDir()
{
  int l = lastDriveLeftPct(), r = lastDriveRightPct();
  if (abs(l - r) > ODOM_STRAIGHT_TOL_PCT) return 0;
  if (l > 0 && r > 0) return +1;
  if (l < 0 && r < 0) return -1;
  return 0;
}

static void commit(float mm) { distMM += mm; }

// end the window: keep the encoder travel, or swap it for the ToF closure
static void closeWindow()
{
  if (!winOpen) return;
  winOpen = false;

  uint16_t endRange = rangeAhead(winDir);
  float encAbs = fabsf(winEncMM);
  if (winStartRange && endRange && encAbs >= ODOM_SLIP_MIN_TRAVEL_MM)
  {
    float closure = (float)winStartRange - (float)endRange;   // mm we actually got closer
    if (closure < ODOM_SLIP_RATIO * encAbs)
    {
      slipping = true;
      slipEvents++;
      commit(winDir * (closure > 0 ? closure : 0.0f));
      return;
    }
  }
  slipping = false;
  commit(winEncMM);
}

void odomInit()
{
  odomReset();
}

void odomReset()
{
  encL.write(0);
  encR.write(0);
  lastAvgCount = 0;
  distMM = 0;
  slipping = false;
  slipEvents = 0;
  winOpen = false;
  stallRefL = stallRefR = 0;
  stallSinceMs = millis();
  stalled = false;
}

void odomUpdate()
{
  unsigned long now = millis();
  long a = avgCount();
  float stepMM = countsToMM(a - lastAvgCount);
  lastAvgCount = a;

  // ---- slip-checked distance ----
  int dir = straightDir();
  if (winOpen && dir != winDir) closeWindow();      // stopped / turned / reversed
  if (dir != 0 && !winOpen)
  {
    winOpen = true;
    winDir = dir;
    winStartMs = now;
    winEncMM = 0;
    winStartRange = rangeAhead(dir);
  }

  if (winOpen)
  {
    winEncMM += stepMM;
    if (now - winStartMs >= ODOM_SLIP_WINDOW_MS) closeWindow();
  }
  else
  {
    commit(stepMM);     // turning / stopped: no check, take the encoders as-is
  }

  // ---- stall ----
  long l = encLeftCount(), r = encRightCount();
  bool commanded = abs(lastDriveLeftPct())  >= ODOM_STALL_MIN_PCT ||
                   abs(lastDriveRightPct()) >= ODOM_STALL_MIN_PCT;
  if (!commanded || l != stallRefL || r != stallRefR)
  {
    stallRefL = l; stallRefR = r;
    stallSinceMs = now;
    stalled = false;
  }
  else if (now - stallSinceMs >= ODOM_STALL_MS)
  {
    stalled = true;
  }
}

float odomRawDistanceMM() { return countsToMM(avgCount()); }
float odomDistanceMM()    { return distMM + (winOpen ? winEncMM : 0.0f); }   // provisional mid-window
bool  odomSlipping()      { return slipping; }
bool  odomStalled()       { return stalled; }
unsigned odomSlipEvents() { return slipEvents; }
