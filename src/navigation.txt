#include "navigation.h"
#include "config.h"
#include "drive.h"
#include "tof.h"
#include "ir_sensors.h"
#include "weight_detect.h"
#include "collection.h"

// ============================================================================
//  navigation.cpp
// renamed to txt while testing servo since errors occurred here
//  Same core control shapes throughout this project:
//      DISCRETE decisions -> bang-bang with hysteresis:
//          FORWARD / TURN_L / TURN_R / ESCAPE, flip detection, escalating
//          escapes.
//      CONTINUOUS tracking -> proportional / PD:
//          - caution veer: proportional lean away from walls entering
//            CAUTION_MM (top-front ToF pair)
//          - side nudge: step away from a side IR getting too close
//          - APPROACH: PD on the low-front-pair imbalance to centre a
//            weight candidate
//
//  PICKUP - THE REAL MECHANISM (this is what's new vs earlier nav-only
//  builds, which only had a placeholder "drive through it"):
//      APPROACH  close at 55%, PD-centring the weight dead ahead
//      PICKUP    weight passes the trigger distance under the low ToF pair
//                -> STOP, call collection_start() once, hold position while
//                collection_busy() (your partner's crane/magnet FSM) runs
//                the full pickup -> drop -> rest cycle, then move on.
//                We can't confirm the magnets actually grabbed anything at
//                ground level (a dummy weight won't be attracted) - see
//                funnel_sensor.cpp for the ACTUAL real/dummy classification,
//                which happens later, at the funnel end. So PICKUP always
//                suppresses re-detection of that spot afterward, win or lose.
//      REPOSITION  turn away from the spot just worked (no reverse needed -
//                the crane operates in place, we never drove into anything)
//                then resume searching.
//
//  SCAN: nothing found for SCAN_TRIGGER_MS of cruising -> spin on the spot
//  for SCAN_MAX_MS (timed: no IMU here to measure the angle - see "IMU HOOK"
//  below for where that would plug in instead).
//
//  Priority: rear guard > pickup > approach > hard avoid > caution veer /
//            side nudge > cruise
// ============================================================================

const int MODE_FORWARD    = 0;
const int MODE_TURN_LEFT  = 1;
const int MODE_TURN_RIGHT = 2;
const int MODE_ESCAPE     = 3;
const int MODE_APPROACH   = 4;
const int MODE_PICKUP     = 5;
const int MODE_REPOSITION = 6;
const int MODE_SCAN       = 7;

static int mode = MODE_FORWARD;
static unsigned long modeStart = 0;

static int  lastTurn = 0;
static unsigned long lastTurnEnd = 0;
static int  flipCount = 0;
static int  escapeSpinDir = 1;
static unsigned long escapeSpinMs = ESCAPE_SPIN_MS;

static unsigned long lastFindOrEvent = 0;
static float approachErrPrev = 0;
static int scanDir = 1;
static int repositionDir = 1;
static bool pickupStarted = false;

static void setMode(int m) { mode = m; modeStart = millis(); }

const char* modeName()
{
  switch (mode)
  {
    case MODE_FORWARD:    return "FORWARD";
    case MODE_TURN_LEFT:  return "TURN_L";
    case MODE_TURN_RIGHT: return "TURN_R";
    case MODE_ESCAPE:     return "ESCAPE";
    case MODE_APPROACH:   return "APPROACH";
    case MODE_PICKUP:     return "PICKUP";
    case MODE_REPOSITION: return "REPOSITION";
    default:              return "SCAN";
  }
}

static void requestTurn(int dir)
{
  if (lastTurn != 0 && dir != lastTurn && (millis() - lastTurnEnd) < FLIP_WINDOW_MS)
    flipCount++;
  else
    flipCount = 0;

  if (flipCount >= FLIPS_BEFORE_ESCAPE)
  {
    escapeSpinDir = lastTurn;
    escapeSpinMs  = ESCAPE_SPIN_MS + (unsigned long)flipCount * 400;
    if (escapeSpinMs > 3000) escapeSpinMs = 3000;
    setMode(MODE_ESCAPE);
    return;
  }
  lastTurn = dir;
  setMode(dir < 0 ? MODE_TURN_LEFT : MODE_TURN_RIGHT);
}

// ---------------------------------------------------------------------------
//  Obstacle picture. Top-front pair + angled corner pair define an
//  obstacle. A low-pair-only return is a weight candidate, not a wall.
// ---------------------------------------------------------------------------
static bool obstacleLeft()
{
  return (tofFL > 0 && tofFL < NEAR_MM) || cornerNearLeft();
}
static bool obstacleRight()
{
  return (tofFR > 0 && tofFR < NEAR_MM) || cornerNearRight();
}
static bool leftOpen()
{
  return (tofFL == 0 || tofFL > FAR_MM) && !cornerNearLeft();
}
static bool rightOpen()
{
  return (tofFR == 0 || tofFR > FAR_MM) && !cornerNearRight();
}

static int cautionVeer()
{
  int veer = 0;
  if (tofFL > 0 && tofFL < CAUTION_MM && tofFL >= NEAR_MM)
    veer += (int)(KC_AVOID * (CAUTION_MM - tofFL));
  if (tofFR > 0 && tofFR < CAUTION_MM && tofFR >= NEAR_MM)
    veer -= (int)(KC_AVOID * (CAUTION_MM - tofFR));
  if (veer >  CAUTION_VEER_MAX) veer =  CAUTION_VEER_MAX;
  if (veer < -CAUTION_VEER_MAX) veer = -CAUTION_VEER_MAX;
  return veer;
}

void navigationInit()
{
  lastFindOrEvent = millis();
  setMode(MODE_FORWARD);
}

// ---------------------------------------------------------------------------
void navigationUpdate()
{
  unsigned long held = millis() - modeStart;

  // ================= decide =================
  switch (mode)
  {
    case MODE_FORWARD:
    {
      if (obstacleLeft() && obstacleRight())
      {
        escapeSpinDir = (tofFR >= tofFL) ? 1 : -1;
        escapeSpinMs  = ESCAPE_SPIN_MS;
        setMode(MODE_ESCAPE);
      }
      else if (obstacleLeft())  requestTurn(+1);
      else if (obstacleRight()) requestTurn(-1);
      else if (weightFound)
      {
        lastFindOrEvent = millis();
        approachErrPrev = 0;
        setMode(MODE_APPROACH);
      }
#if USE_SCAN
      // IMU HOOK: with a heading reference, this could scan a bounded sweep
      // (e.g. +/-60 degrees) instead of a blind, un-measured spin.
      else if (millis() - lastFindOrEvent > SCAN_TRIGGER_MS)
      {
        scanDir = -scanDir;
        setMode(MODE_SCAN);
      }
#endif
      break;
    }

    case MODE_TURN_RIGHT:
      if (held > MAX_TURN_MS)
      {
        escapeSpinDir = 1;
        escapeSpinMs  = ESCAPE_SPIN_MS + 800;
        setMode(MODE_ESCAPE);
      }
      else if (held > MIN_TURN_MS && leftOpen())
      {
        lastTurnEnd = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_TURN_LEFT:
      if (held > MAX_TURN_MS)
      {
        escapeSpinDir = -1;
        escapeSpinMs  = ESCAPE_SPIN_MS + 800;
        setMode(MODE_ESCAPE);
      }
      else if (held > MIN_TURN_MS && rightOpen())
      {
        lastTurnEnd = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_ESCAPE:
      if (held > ESCAPE_REV_MS + escapeSpinMs)
      {
        flipCount = 0; lastTurn = 0; lastTurnEnd = millis();
        lastFindOrEvent = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_SCAN:
      if (weightFound)
      {
        lastFindOrEvent = millis();
        approachErrPrev = 0;
        setMode(MODE_APPROACH);
      }
      else if (held > SCAN_MAX_MS || obstacleLeft() || obstacleRight())
      {
        lastFindOrEvent = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_APPROACH:
    {
      if (obstacleLeft() || obstacleRight())        // walls outrank prizes
      {
        if (obstacleLeft() && obstacleRight())
        {
          escapeSpinDir = (tofFR >= tofFL) ? 1 : -1;
          escapeSpinMs  = ESCAPE_SPIN_MS;
          setMode(MODE_ESCAPE);
        }
        else requestTurn(obstacleLeft() ? +1 : -1);
        break;
      }

      if (weightFound && weightDistMM > 0 && weightDistMM < PICKUP_TRIGGER_MM)
      {
        repositionDir = (weightSide < 0) ? +1 : -1;   // peel away from its side
        pickupStarted = false;
        setMode(MODE_PICKUP);
        break;
      }

      if (!weightFound && held > 400)
        setMode(MODE_FORWARD);
      break;
    }

    case MODE_PICKUP:
      if (!pickupStarted)
      {
        collection_start();      // your partner's crane/magnet FSM
        pickupStarted = true;
      }
      else if (!collection_busy())
      {
        pickupAttempts++;
        Serial.print(">>> PICKUP ATTEMPT #"); Serial.println(pickupAttempts);
        Serial.println("    (real vs dummy confirmed separately - see funnel telemetry)");
        // can't confirm the grab from here (a dummy won't be attracted to
        // the magnets) - suppress regardless so we don't loop on this spot
        suppressTargetFor(TARGET_SUPPRESS_MS);
        lastFindOrEvent = millis();
        lastTurn = 0; flipCount = 0;
        setMode(MODE_REPOSITION);
      }
      break;

    case MODE_REPOSITION:
      if (held > REPOSITION_TURN_MS)
        setMode(MODE_FORWARD);
      break;
  }

  // ================= act =================
  switch (mode)
  {
    case MODE_FORWARD:
    {
      int steer = cautionVeer();
      if (sideNearLeft())  steer += SIDE_NUDGE_PCT;
      if (sideNearRight()) steer -= SIDE_NUDGE_PCT;
      drive(CRUISE_SPEED_PCT + steer, CRUISE_SPEED_PCT - steer);
      break;
    }

    case MODE_TURN_LEFT:   turnLeft();   break;
    case MODE_TURN_RIGHT:  turnRight();  break;

    case MODE_ESCAPE:
      if (held < ESCAPE_REV_MS)
      {
        if (rearBlocked()) stopMotors();
        else               driveReverse();
      }
      else if (escapeSpinDir < 0) turnLeft();
      else                        turnRight();
      break;

    case MODE_SCAN:
      drive(scanDir * SCAN_SPIN_PCT, -scanDir * SCAN_SPIN_PCT);
      break;

    case MODE_APPROACH:
    {
      int steer;
      if (weightSide == 0 && tofBL > 0 && tofBR > 0)
      {
        float err  = (float)tofBL - (float)tofBR;
        float dErr = err - approachErrPrev;
        approachErrPrev = err;
        float s = KP_APPROACH * err + KD_APPROACH * dErr;
        if (s >  APPROACH_STEER_MAX) s =  APPROACH_STEER_MAX;
        if (s < -APPROACH_STEER_MAX) s = -APPROACH_STEER_MAX;
        steer = (int)s;
      }
      else
      {
        steer = (weightSide < 0) ? -ONE_SIDE_ARC_PCT : +ONE_SIDE_ARC_PCT;
        approachErrPrev = 0;
      }
      drive(APPROACH_SPEED_PCT + steer, APPROACH_SPEED_PCT - steer);
      break;
    }

    case MODE_PICKUP:
      stopMotors();     // hold still while the crane works
      break;

    case MODE_REPOSITION:
      // IMU HOOK: with a heading reference this could turn a measured
      // angle instead of a fixed time.
      drive(repositionDir * REPOSITION_SPEED_PCT, -repositionDir * REPOSITION_SPEED_PCT);
      break;
  }

  // ---- universal rear guard: never reverse into something we can see ----
  if (isReversing() && rearBlocked())
    stopMotors();

  // BEACON HOOK: once an IR beacon is added, a "return to base" mode would
  // slot in here - e.g. triggered after N pickups or with time running low -
  // steering on beacon signal instead of the ToF/IR obstacle picture.
}
