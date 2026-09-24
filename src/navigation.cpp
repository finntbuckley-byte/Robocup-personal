#include "navigation.h"
#include "config.h"
#include "drive.h"
#include "tof.h"
#include "x8.h"
#include "ir_sensors.h"
#include "weight_detect.h"
#include "collection.h"
#include "round.h"

// ============================================================================
//  navigation.cpp  -  built only by [env:nav] (see platformio.ini).
//
//  Same core control shapes throughout this project:
//      DISCRETE decisions -> bang-bang with hysteresis:
//          FORWARD / TURN_L / TURN_R / ESCAPE, flip detection, escalating
//          escapes.
//      CONTINUOUS tracking -> proportional / PD:
//          - caution veer: proportional lean away from whichever half of the
//            SEN0628 obstacle band enters CAUTION_MM
//          - side nudge: step away from a side IR getting too close
//          - APPROACH: PD on the bottom-ToF-pair imbalance to centre a
//            weight candidate
//
//  OBSTACLES come from the SEN0628 8x8 (x8.h), split into left/right halves
//  of its obstacle band. If its frame goes stale (x8Fresh() false) the front
//  is UNKNOWN: FORWARD crawls at BLIND_SPEED_PCT with the side IR nudge only,
//  turns end on time, and no weight candidates are reported.
//
//  PICKUP:
//      APPROACH  close at APPROACH_SPEED_PCT, PD-centring the weight
//      PICKUP    weight inside PICKUP_TRIGGER_MM -> STOP, collection_start()
//                once, hold still while collection_busy(), then move on
//                (or give up after PICKUP_TIMEOUT_MS). The grab can't be
//                confirmed here - funnel_sensor.cpp classifies what actually
//                arrives - so the spot is suppressed afterwards either way.
//      REPOSITION  turn away from the spot just worked, then resume.
//
//  ROUND (round.h): weights are only approached while roundWantsWeights() -
//  i.e. under the 3-target cap and before RETURN_HOME_AT_MS. After that the
//  robot keeps avoiding obstacles but ignores weights (RETURN_HOME needs the
//  IMU - see IMU HOOK below).
//
//  SCAN: nothing found for SCAN_TRIGGER_MS -> spin on the spot for
//  SCAN_MAX_MS (timed - see IMU HOOK).
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
//  Obstacle picture - SEN0628 obstacle band, left/right halves. A bottom-
//  pair-only return is a weight candidate (weight_detect.cpp), not a wall.
// ---------------------------------------------------------------------------
static inline bool isNear(uint16_t mm) { return mm > 0 && mm < NEAR_MM; }
static inline bool isOpen(uint16_t mm) { return mm == 0 || mm > FAR_MM; }
static inline uint16_t room(uint16_t mm) { return mm == 0 ? 0xFFFF : mm; }

static bool obstacleLeft()  { return x8Fresh() && isNear(x8LeftMM()); }
static bool obstacleRight() { return x8Fresh() && isNear(x8RightMM()); }
static bool leftOpen()      { return x8Fresh() && isOpen(x8LeftMM()); }
static bool rightOpen()     { return x8Fresh() && isOpen(x8RightMM()); }

// spin toward whichever half has more room
static int roomierSide() { return room(x8RightMM()) >= room(x8LeftMM()) ? +1 : -1; }

static int cautionVeer()
{
  if (!x8Fresh()) return 0;
  uint16_t l = x8LeftMM(), r = x8RightMM();
  int veer = 0;
  if (l > 0 && l < CAUTION_MM && l >= NEAR_MM) veer += (int)(KC_AVOID * (CAUTION_MM - l));
  if (r > 0 && r < CAUTION_MM && r >= NEAR_MM) veer -= (int)(KC_AVOID * (CAUTION_MM - r));
  if (veer >  CAUTION_VEER_MAX) veer =  CAUTION_VEER_MAX;
  if (veer < -CAUTION_VEER_MAX) veer = -CAUTION_VEER_MAX;
  return veer;
}

static void startApproach()
{
  lastFindOrEvent = millis();
  approachErrPrev = 0;
  setMode(MODE_APPROACH);
}

void navigationInit()
{
  lastTurn = 0; flipCount = 0;
  lastFindOrEvent = millis();
  setMode(MODE_FORWARD);
}

// ---------------------------------------------------------------------------
void navigationUpdate()
{
  unsigned long held = millis() - modeStart;
  bool wantWeights = roundWantsWeights();

  // IMU HOOK: once the IMU is back, roundWantsHome() is where a RETURN_HOME
  // mode takes over - heading back toward the start corner (heading zeroed at
  // roundJustStarted()), then a DELIVER mode opens the rear flap and calls
  // noteDelivered(). Until then the robot keeps roaming and avoiding, but
  // stops collecting (see wantWeights) so it never exceeds the 3-target cap.
  // BEACON HOOK: fallback homing on an IR beacon slots in the same place.

  // ================= decide =================
  switch (mode)
  {
    case MODE_FORWARD:
    {
      if (obstacleLeft() && obstacleRight())
      {
        escapeSpinDir = roomierSide();
        escapeSpinMs  = ESCAPE_SPIN_MS;
        setMode(MODE_ESCAPE);
      }
      else if (obstacleLeft())  requestTurn(+1);
      else if (obstacleRight()) requestTurn(-1);
      else if (wantWeights && weightFound) startApproach();
#if USE_SCAN
      // IMU HOOK: with a heading reference, this could scan a bounded sweep
      // (e.g. +/-60 degrees) instead of a blind, un-measured spin.
      else if (wantWeights && x8Fresh() && millis() - lastFindOrEvent > SCAN_TRIGGER_MS)
      {
        scanDir = -scanDir;
        setMode(MODE_SCAN);
      }
#endif
      break;
    }

    case MODE_TURN_RIGHT:
      if (held > MAX_TURN_MS && x8Fresh())
      {
        escapeSpinDir = 1;
        escapeSpinMs  = ESCAPE_SPIN_MS + 800;
        setMode(MODE_ESCAPE);
      }
      // stale 8x8: can't see whether it's open, so end on time
      else if (held > MIN_TURN_MS && (leftOpen() || !x8Fresh()))
      {
        lastTurnEnd = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_TURN_LEFT:
      if (held > MAX_TURN_MS && x8Fresh())
      {
        escapeSpinDir = -1;
        escapeSpinMs  = ESCAPE_SPIN_MS + 800;
        setMode(MODE_ESCAPE);
      }
      else if (held > MIN_TURN_MS && (rightOpen() || !x8Fresh()))
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
      if (wantWeights && weightFound) startApproach();
      else if (held > SCAN_MAX_MS || obstacleLeft() || obstacleRight() || !wantWeights)
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
          escapeSpinDir = roomierSide();
          escapeSpinMs  = ESCAPE_SPIN_MS;
          setMode(MODE_ESCAPE);
        }
        else requestTurn(obstacleLeft() ? +1 : -1);
        break;
      }

      if (!wantWeights) { setMode(MODE_FORWARD); break; }

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
        collection_start();      // partner's crane/magnet FSM
        pickupStarted = true;
      }
      else if (!collection_busy() || held > PICKUP_TIMEOUT_MS)
      {
        pickupAttempts++;
        Serial.print(">>> PICKUP ATTEMPT #"); Serial.print(pickupAttempts);
        Serial.println(held > PICKUP_TIMEOUT_MS ? " (TIMED OUT waiting for crane)" : "");
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
      int speed = x8Fresh() ? CRUISE_SPEED_PCT : BLIND_SPEED_PCT;
      int steer = cautionVeer();
      if (sideNearLeft())  steer += SIDE_NUDGE_PCT;
      if (sideNearRight()) steer -= SIDE_NUDGE_PCT;
      drive(speed + steer, speed - steer);
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
}
