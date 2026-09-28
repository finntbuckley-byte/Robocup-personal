#include "navigation.h"
#include "config.h"
#include "drive.h"
#include "tof.h"
#include "x8.h"
#include "ir_sensors.h"
#include "weight_detect.h"
#include "collection.h"
#include "round.h"
#include "funnel_sensor.h"   // inductiveMetalNow()
#include "imu.h"
#include "odometry.h"        // odomStalled() for the APPROACH progress check

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
//  PICKUP - triggered by the INDUCTIVE sensor (front-on at the notch, 40mm
//  up; only reads metal when steel is within ~7mm = weight seated in the
//  notch). Bench data 2026-09-25: steel upright -> metal; plain plastic,
//  steel lying -> not metal. So one check = "real weight, in position".
//      APPROACH  steer on the bottom ToF pair (PD when both see it, else a
//                one-sided arc). They see a weight from ~14cm out but lose it
//                as it enters the notch, so:
//      CREEP     weight lost while close (< CREEP_START_MM) -> creep
//                straight through the blind gap for up to CREEP_MAX_MS.
//      PICKUP    inductive reads metal (debounced) in APPROACH or CREEP ->
//                STOP, run the crane, hold while collection_busy(). After the
//                cycle: metal GONE from the notch = success (noteCollected);
//                metal STILL there = the grab missed -> retry up to
//                MAX_PICKUP_TRIES, then give up.
//      REJECT    two triggers, same outcome (reverse REJECT_REVERSE_MS,
//                pivot away, suppress the spot):
//                  - lyingWeightConfirmed(): the notch ToF sees something but
//                    the baseplate-top ToF doesn't (bench-confirmed blind to
//                    a weight lying on its side) - checked in FORWARD,
//                    APPROACH and CREEP, so a lying weight is rejected as
//                    soon as it's seen rather than after creeping into it.
//                  - CREEP times out with no metal ever seen (dummy/nothing).
//                No longer gated on the inductive sensor - that's PICKUP-only
//                now (metalConfirmed()).
//      REPOSITION  after a pickup, turn away from the spot, then resume.
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
const int MODE_CREEP      = 8;
const int MODE_REJECT     = 9;

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
static bool pickupIsThird = false;    // decided once at startPickup(), held through retries
static int  pickupTries = 0;
static uint16_t lastWeightDist = 0;   // last distance the bottom pair saw the candidate at
static int  lastWeightSide = 0;
static int  rejectDir = 1;
static int  rejectCount = 0;
static unsigned long metalSince = 0;
static unsigned long lyingSince = 0;

static void setMode(int m) { mode = m; modeStart = millis(); }

static float holdTarget = 0;         // heading FORWARD is holding (imu.h: + = right)

// After giving up on a weight (grab missed MAX_PICKUP_TRIES times, or the
// crane timed out) the trigger is locked out until the notch has read CLEAR
// for METAL_REARM_CLEAR_MS - otherwise a weight that stays in the notch
// (jammed, can't be gripped) restarts the pickup forever. Seen on blocks 28/9:
// gave up, back to FORWARD, same weight -> 3rd pickup immediately.
static unsigned long pickupCycleStart = 0;   // this crane cycle's start
static unsigned long metalLeftAt = 0;        // first moment the notch read clear in it (0 = not yet)
static bool metalLockout = false;
static unsigned long metalClearSince = 0;

// inductive reads metal continuously for INDUCTIVE_CONFIRM_MS
static bool metalConfirmed()
{
  if (metalLockout)
  {
    if (inductiveMetalNow()) { metalClearSince = 0; return false; }
    if (metalClearSince == 0) metalClearSince = millis();
    if (millis() - metalClearSince < METAL_REARM_CLEAR_MS) return false;
    metalLockout = false;
    Serial.println(">>> pickup trigger re-armed (notch clear)");
  }
  if (!inductiveMetalNow()) { metalSince = 0; return false; }
  if (metalSince == 0) metalSince = millis();
  return millis() - metalSince >= INDUCTIVE_CONFIRM_MS;
}

static int roomierSide();

// TOF_TOP (baseplate-top, proposed - see config.h) is bench-confirmed blind
// to a weight lying on its side; TOF_UPRIGHT (the notch ToF) still sees it.
// So bottom-sees-something + top-sees-nothing, held steadily, means "an
// object is here but not standing up" - reject it before creeping it
// through the funnel. If the top sensor isn't up (not yet fitted, stale,
// mid-recovery) this never fires, rather than treating "no top sensor" as
// "definitely lying down" - that's what caused real weights to get rejected
// before this sensor existed.
static bool lyingWeightConfirmed()
{
  if (!tofOk(TOF_TOP) || !tofOk(TOF_UPRIGHT)) { lyingSince = 0; return false; }

  bool bottomSees = tofUpright < BOTTOM_PRESENT_MM;
  bool topSees    = tofTop     < TOP_PRESENT_MM;

  if (!bottomSees || topSees) { lyingSince = 0; return false; }

  if (lyingSince == 0) lyingSince = millis();
  return millis() - lyingSince >= LYING_CONFIRM_MS;
}

int rejectedCount() { return rejectCount; }

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
    case MODE_CREEP:      return "CREEP";
    case MODE_REJECT:     return "REJECT";
    default:              return "SCAN";
  }
}

// Corners: a wall on one side, turn away, the other wall appears -> the old
// code turned straight back (tall-box test 28/9: TURN_L -> TURN_R). Within
// FLIP_WINDOW_MS of the last turn, an opposite request is overridden: KEEP
// turning the way we were, so the robot rotates out of the corner in one
// direction. The second such flip escalates to ESCAPE (reverse + spin).
static void requestTurn(int dir)
{
  if (lastTurn != 0 && dir != lastTurn && (millis() - lastTurnEnd) < FLIP_WINDOW_MS)
  {
    flipCount++;
    dir = lastTurn;               // commit to the original direction
  }
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
static inline bool clearForScan(uint16_t mm) { return mm == 0 || mm > SCAN_CLEAR_MM; }

// APPROACH progress check (see config.h APPROACH_PROGRESS_MS)
static uint16_t approachRefDist = 0;
static unsigned long approachRefAt = 0;
static int approachGiveUps = 0;
int approachGiveUpCount() { return approachGiveUps; }

static void abandonApproach(const char *why)
{
  approachGiveUps++;
  Serial.print(">>> APPROACH abandoned #"); Serial.print(approachGiveUps);
  Serial.print(" - "); Serial.println(why);
  suppressTargetFor(APPROACH_GIVEUP_SUPPRESS_MS);
  lastFindOrEvent = millis();
  setMode(MODE_FORWARD);
}

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
  lastWeightDist = 0;
  approachRefDist = 0;
  setMode(MODE_APPROACH);
}

static void startPickup()
{
  repositionDir = (lastWeightSide < 0) ? +1 : -1;   // peel away from its side
  pickupStarted = false;
  // decided once, before the crane moves, so a MISS retry doesn't re-check
  // targetsOnBoard() mid-cycle and flip which behaviour this pickup uses
  pickupIsThird = targetsOnBoard() >= MAX_TARGETS_ON_BOARD - 1;
  setMode(MODE_PICKUP);
}

static void startReject(const char *why)
{
  rejectCount++;
  rejectDir = (lastWeightSide < 0) ? +1 : -1;
  Serial.print(">>> REJECT #"); Serial.print(rejectCount);
  Serial.print(" - "); Serial.println(why);
  setMode(MODE_REJECT);
}

// walls outrank prizes: returns true if it switched to a turn/escape
static bool avoidIfBlocked()
{
  if (!obstacleLeft() && !obstacleRight()) return false;
  if (obstacleLeft() && obstacleRight())
  {
    escapeSpinDir = roomierSide();
    escapeSpinMs  = ESCAPE_SPIN_MS;
    setMode(MODE_ESCAPE);
  }
  else requestTurn(obstacleLeft() ? +1 : -1);
  return true;
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
      // a weight can end up in the notch without an approach (drove into it)
      if (wantWeights && metalConfirmed()) { pickupTries = 0; lastWeightSide = 0; startPickup(); }
      else if (wantWeights && lyingWeightConfirmed()) startReject("lying weight (top/bottom ToF)");
      else if (obstacleLeft() && obstacleRight())
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
      // only with clear space on both sides, and spin toward the roomier one
      else if (wantWeights && x8Fresh() && millis() - lastFindOrEvent > SCAN_TRIGGER_MS &&
               clearForScan(x8LeftMM()) && clearForScan(x8RightMM()))
      {
        scanDir = roomierSide();
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
      // stale 8x8: can't see whether it's open, so end on time. Otherwise end
      // only when the blocked side is open AND the other side isn't close
      // (else FORWARD would immediately start the opposite turn)
      else if (held > MIN_TURN_MS && ((leftOpen() && !obstacleRight()) || !x8Fresh()))
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
      else if (held > MIN_TURN_MS && ((rightOpen() && !obstacleLeft()) || !x8Fresh()))
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
      if (wantWeights && metalConfirmed()) { pickupTries = 0; lastWeightSide = 0; startPickup(); }
      else if (wantWeights && weightFound) startApproach();
      else if (held > SCAN_MAX_MS || obstacleLeft() || obstacleRight() || !wantWeights)
      {
        lastFindOrEvent = millis();
        setMode(MODE_FORWARD);
      }
      break;

    case MODE_APPROACH:
    {
      if (avoidIfBlocked()) break;
      if (!wantWeights) { setMode(MODE_FORWARD); break; }

      if (metalConfirmed()) { pickupTries = 0; startPickup(); break; }
      if (lyingWeightConfirmed()) { startReject("lying weight (top/bottom ToF)"); break; }

      if (odomStalled() && held > APPROACH_PROGRESS_MS) { abandonApproach("tracks stalled"); break; }

      if (weightFound)
      {
        lastWeightDist = weightDistMM;
        lastWeightSide = weightSide;

        // progress: a real weight gets closer; a wall seen at an angle doesn't
        if (approachRefDist == 0) { approachRefDist = weightDistMM; approachRefAt = millis(); }
        else if (millis() - approachRefAt >= APPROACH_PROGRESS_MS)
        {
          if ((int)approachRefDist - (int)weightDistMM < APPROACH_MIN_CLOSE_MM)
          {
            abandonApproach("candidate not getting closer (wall at an angle / blocked)");
            break;
          }
          approachRefDist = weightDistMM;
          approachRefAt = millis();
        }
      }
      else if (lastWeightDist > 0 && lastWeightDist < CREEP_START_MM)
      {
        setMode(MODE_CREEP);            // lost it at the notch mouth - creep through the blind gap
      }
      else if (held > 400)
        setMode(MODE_FORWARD);          // lost it far out - give up
      break;
    }

    case MODE_CREEP:
      if (avoidIfBlocked()) break;
      if (!wantWeights) { setMode(MODE_FORWARD); break; }
      if (metalConfirmed()) { pickupTries = 0; startPickup(); }
      else if (lyingWeightConfirmed()) startReject("lying weight (top/bottom ToF)");
      else if (weightFound && weightDistMM >= CREEP_START_MM) startApproach();   // re-acquired further out
      else if (held > CREEP_MAX_MS) startReject("no metal at the notch (dummy / lying weight / nothing)");
      break;

    case MODE_PICKUP:
      if (!pickupStarted)
      {
        collection_start(pickupIsThird);      // partner's crane/magnet FSM
        pickupStarted = true;
        pickupTries++;
        pickupCycleStart = millis();
        metalLeftAt = 0;
      }
      // DATA ONLY (no behaviour change yet): when did the metal first leave
      // the notch? Real lifts 28/9 held metal steadily on the way down and
      // only lost it ~1.2s in (the lift). A weight pushed out by the arm would
      // leave EARLIER. Floor logs decide the cut-off / whether option 3 (a
      // crane "lift started" signal) is needed.
      if (pickupStarted && metalLeftAt == 0 && !inductiveMetalNow())
        metalLeftAt = millis();
      else if (!collection_busy() || held > PICKUP_TIMEOUT_MS)
      {
        pickupAttempts++;
        bool timedOut = held > PICKUP_TIMEOUT_MS;
        bool stillThere = inductiveMetalNow();   // metal still in the notch = the grab missed

        Serial.print(">>> PICKUP ATTEMPT #"); Serial.print(pickupAttempts);
        if (timedOut)        Serial.print(" - TIMED OUT waiting for crane");
        else if (stillThere) Serial.print(" - MISSED (metal still in the notch)");
        else                 Serial.print(" - OK (weight gone from the notch)");
        Serial.print(", metal left at ");
        if (metalLeftAt) { Serial.print((metalLeftAt - pickupCycleStart) / 1000.0f, 2); Serial.println(" s"); }
        else               Serial.println("never");

        if (!timedOut && stillThere && pickupTries < MAX_PICKUP_TRIES)
        {
          pickupStarted = false;          // retry in place
          modeStart = millis();
          break;
        }
        if (!timedOut && !stillThere) noteCollected();
        else
        {
          metalLockout = true;            // leave this weight behind before triggering again
          metalClearSince = 0;
          Serial.println(">>> giving up on this weight - trigger locked until the notch is clear");
        }

        suppressTargetFor(TARGET_SUPPRESS_MS);
        lastFindOrEvent = millis();
        lastTurn = 0; flipCount = 0;
        setMode(MODE_REPOSITION);
      }
      break;

    case MODE_REJECT:
      if (held > REJECT_REVERSE_MS + REPOSITION_TURN_MS)
      {
        suppressTargetFor(REJECT_SUPPRESS_MS);
        lastFindOrEvent = millis();
        lastTurn = 0; flipCount = 0;
        setMode(MODE_FORWARD);
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
      // heading hold: steering on purpose (or just out of a turn) re-aims it
      if (USE_HEADING_HOLD && imuOk())
      {
        if (steer != 0 || millis() - modeStart < HEADING_HOLD_SETTLE_MS)
          holdTarget = imuHeadingDeg();
        else
          steer = headingHoldSteer(holdTarget);
      }
      drive(speed + steer, speed - steer);
      break;
    }

    case MODE_TURN_LEFT:   turnLeft();   break;
    case MODE_TURN_RIGHT:  turnRight();  break;

    case MODE_ESCAPE:
      if (held < ESCAPE_REV_MS)
      {
        if (rearBlocked()) driveHardStop();
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

    case MODE_CREEP:
      drive(CREEP_SPEED_PCT, CREEP_SPEED_PCT);   // straight - nothing to steer on in the blind gap
      break;

    case MODE_PICKUP:
      stopMotors();     // hold still while the crane works
      break;

    case MODE_REJECT:
      // back out of the notch (rear guard below still applies), then pivot
      // away so the V-notch wall pushes the dummy aside (option C)
      if (held < REJECT_REVERSE_MS)
        drive(-REJECT_REVERSE_PCT, -REJECT_REVERSE_PCT);
      else
        drive(rejectDir * REPOSITION_SPEED_PCT, -rejectDir * REPOSITION_SPEED_PCT);
      break;

    case MODE_REPOSITION:
      // IMU HOOK: with a heading reference this could turn a measured
      // angle instead of a fixed time.
      drive(repositionDir * REPOSITION_SPEED_PCT, -repositionDir * REPOSITION_SPEED_PCT);
      break;
  }

  // ---- universal rear guard: never reverse into something we can see ----
  if (isReversing() && rearBlocked())
    driveHardStop();      // no soft stop when about to back into something
}
