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
#include "homing.h"
#include "home_detour.h"
#include "pose.h"

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
//      CREEP     weight lost while close (< CREEP_START_MM), or the notch ToF
//                sees something (FORWARD/APPROACH - stack_test's SEARCH ->
//                CREEP) -> creep straight. Blind gap (notch hasn't seen
//                anything): up to CREEP_MAX_MS. Once the notch has seen it:
//                up to navTuning().creepCapMs, back to FORWARD if it leaves
//                the notch's view.
//      PICKUP    inductive reads metal (debounced) in APPROACH or CREEP ->
//                STOP, run the crane, hold while collection_busy(). After the
//                cycle: metal GONE from the notch = success (noteCollected);
//                metal STILL there = the grab missed -> retry up to
//                MAX_PICKUP_TRIES, then give up.
//      REJECT    Two triggers, same reverse/pivot/suppress (timings now live-
//                tunable - see navTuning()): (1) CREEP times out with no
//                metal ever seen (dummy/nothing). (2) lyingWeightConfirmed()
//                (notch ToF sees something but the baseplate-top ToF never
//                confirms, held patiently - acted on in CREEP only), same
//                logic as stack_test.cpp (USE_LYING_WEIGHT_REJECT in
//                config.h, BENCH_TODO.md 2h/2i). Neither
//                is gated on the inductive sensor - that's PICKUP-only now
//                (metalConfirmed()). The move (2026-09-29): reverse
//                REJECT_REVERSE_MM by the encoders, then pivot
//                REJECT_PIVOT_DEG by the IMU - cut short if the bottom pair
//                spots another weight once it has turned far enough.
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
#if USE_LYING_WEIGHT_REJECT
static bool creepNotchSeen = false;         // this CREEP has had the notch ToF see something
static unsigned long notchIgnoreUntil = 0;  // after a REJECT: notch can't restart CREEP until then
#endif

// REJECT phases: reverse by distance, then pivot by angle (see config.h)
static bool  rejectPivoting = false;
static float rejectStartMm = 0;        // odomRawDistanceMM() when the reverse began
static float rejectStartHdg = 0;       // heading when the pivot began
static unsigned long rejectPivotAt = 0;

static float wrap180(float d)
{
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static NavTuning tuning = { REJECT_REVERSE_MM, REJECT_PIVOT_DEG, REPOSITION_TURN_MS,
                             LYING_DISCREPANCY_MM, LYING_CONFIRM_MS,
                             CREEP_HARD_CAP_MS };
NavTuning &navTuning() { return tuning; }

static void setMode(int m) { mode = m; modeStart = millis(); }

static bool homeAligning = false;
static home::Detour homeDetour(HOME_DETOUR_PASS_MM, HOME_DETOUR_CLEAR_MS);
static float holdTarget = 0;         // heading FORWARD is holding (imu.h: + = right)

// After giving up on a weight (grab missed MAX_PICKUP_TRIES times, or the
// crane timed out) the trigger is locked out until the notch has read CLEAR
// for METAL_REARM_CLEAR_MS - otherwise a weight that stays in the notch
// (jammed, can't be gripped) restarts the pickup forever. Seen on blocks 28/9:
// gave up, back to FORWARD, same weight -> 3rd pickup immediately.
static unsigned long pickupCycleStart = 0;   // this crane cycle's start
static bool pickupVerifying = false, pickupClearTracking = false;
static unsigned long pickupVerifyAt = 0, pickupClearAt = 0, pickupSampleAt = 0;
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

#if USE_LYING_WEIGHT_REJECT
// Notch sensing still assists creep. The top sensor is now solely a centre
// weight detector in weight_detect.cpp; it never rejects lying weights.
static bool notchPresent() { return tofOk(TOF_UPRIGHT) && tofUpright > 0 && tofUpright < LYING_NOTCH_MAX_MM; }
static bool centreNear() { return weightCentreActive && weightDistMM < CREEP_START_MM; }
static bool intakePresent() { return notchPresent() || centreNear(); }

// the notch ToF sees something to creep onto (stack_test: SEARCH -> CREEP)
static bool notchCandidate() { return notchPresent() && millis() >= notchIgnoreUntil; }
#endif

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

  if (homingNavigating()) dir = homeDetour.choose(dir);
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

static void startCreep()
{
#if USE_LYING_WEIGHT_REJECT
  creepNotchSeen = intakePresent();
#endif
  setMode(MODE_CREEP);
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
  rejectPivoting = !NAV_RECOVERY_REVERSE_ENABLED;
  rejectStartMm = odomRawDistanceMM();
  rejectPivotAt = millis();
  rejectStartHdg = imuHeadingDeg();
  setMode(MODE_REJECT);
}

// End of a REJECT: suppress the notch briefly before trying again.
static void finishReject()
{
#if USE_LYING_WEIGHT_REJECT
  // the object may still be in view of the notch - don't creep straight back onto it
  notchIgnoreUntil = millis() + NOTCH_IGNORE_MS;
#endif
  lastFindOrEvent = millis();
  lastTurn = 0; flipCount = 0;
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
  homeAligning = false;
  homeDetour.reset();
  lastTurn = 0; flipCount = 0;
  lastFindOrEvent = millis();
  setMode(MODE_FORWARD);
}

// ---------------------------------------------------------------------------
void navigationUpdate()
{
  if (homingUpdate(mode == MODE_PICKUP)) return;
  unsigned long held = millis() - modeStart;
  bool wantWeights = roundWantsWeights() && !homingNavigating();

  // Collection keeps its existing FSM. Homing adds commitment after avoidance
  // so direct-home steering cannot immediately turn back into the obstacle.
  const bool goingHome = homingNavigating();

  // ================= decide =================
  switch (mode)
  {
    case MODE_FORWARD:
    {
      // a weight can end up in the notch without an approach (drove into it)
      if (wantWeights && metalConfirmed()) { pickupTries = 0; lastWeightSide = 0; startPickup(); }
      else if (obstacleLeft() && obstacleRight())
      {
        escapeSpinDir = roomierSide();
        if (goingHome) escapeSpinDir = homeDetour.choose(escapeSpinDir);
        escapeSpinMs  = ESCAPE_SPIN_MS;
        setMode(MODE_ESCAPE);
      }
      else if (obstacleLeft())  requestTurn(+1);
      else if (obstacleRight()) requestTurn(-1);
#if USE_LYING_WEIGHT_REJECT
      else if (wantWeights && notchCandidate()) { lastWeightSide = 0; startCreep(); }   // something at the notch
#endif
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
      if (held > (NAV_RECOVERY_REVERSE_ENABLED ? ESCAPE_REV_MS : 0UL) + escapeSpinMs)
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
#if USE_LYING_WEIGHT_REJECT
      if (notchCandidate() || centreNear()) { startCreep(); break; }   // reached the notch - creep + confirm (stack_test)
#endif

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
        startCreep();                   // lost it at the notch mouth - creep through the blind gap
      }
      else if (held > 400)
        setMode(MODE_FORWARD);          // lost it far out - give up
      break;
    }

    case MODE_CREEP:
      if (avoidIfBlocked()) break;
      if (!wantWeights) { setMode(MODE_FORWARD); break; }
      if (metalConfirmed()) { pickupTries = 0; startPickup(); break; }
#if USE_LYING_WEIGHT_REJECT
      // Metal wins, then candidate loss, then the hard cap. No
      // re-acquire -> APPROACH once the notch has it: APPROACH would hand
      // straight back to CREEP and restart the cap every bounce.
      if (intakePresent()) creepNotchSeen = true;
      if (creepNotchSeen)
      {
        if (!intakePresent()) { lastFindOrEvent = millis(); setMode(MODE_FORWARD); break; }   // left the notch FoV
        if (held > tuning.creepCapMs) startReject("creep hard cap - no metal (dummy?)");
        break;
      }
#endif
      // blind gap - notch hasn't seen anything yet
      if (weightFound && weightDistMM >= CREEP_START_MM) { startApproach(); break; }   // re-acquired further out
      if (held > CREEP_MAX_MS) startReject("no metal at the notch (dummy / lying weight / nothing)");
      break;

    case MODE_PICKUP:
    {
      const unsigned long now = millis();
      const bool metalNow = inductiveMetalNow(); // one consistent sample for this decision
      if (!pickupStarted)
      {
        collection_start(pickupIsThird);
        pickupStarted = true;
        pickupTries++;
        pickupCycleStart = now;
        pickupVerifying = pickupClearTracking = false;
        metalLeftAt = 0;
        driveHardStop();
      }
      // First departure is telemetry only, not proof that the pickup succeeded.
      if (metalLeftAt == 0 && !metalNow) metalLeftAt = now;

      const bool timedOut = collection_busy() && now - pickupCycleStart > PICKUP_TIMEOUT_MS;
      if (collection_busy() && !timedOut) break;
      if (timedOut) collection_stop_motion(); // never drive away with a timed-out arm moving

      if (!pickupVerifying)
      {
        pickupVerifying = true;
        pickupVerifyAt = pickupSampleAt = now;
        pickupClearTracking = false;
        Serial.printf(">>> PICKUP_VERIFY target_try=%d/%d metal=%d\n",
                      pickupTries, MAX_PICKUP_TRIES, metalNow);
      }
      if (metalNow || now - pickupSampleAt > PICKUP_VERIFY_MAX_SAMPLE_GAP_MS)
        pickupClearTracking = false;
      if (!metalNow && !pickupClearTracking)
      {
        pickupClearTracking = true;
        pickupClearAt = now;
      }
      pickupSampleAt = now;
      const unsigned long clearMs = pickupClearTracking ? now - pickupClearAt : 0;
      const bool confirmed = !timedOut && !metalNow && pickupClearTracking &&
                             now - pickupVerifyAt >= PICKUP_VERIFY_MIN_MS &&
                             clearMs >= PICKUP_CLEAR_CONFIRM_MS;
      if (!timedOut && !confirmed && now - pickupVerifyAt < PICKUP_VERIFY_TIMEOUT_MS)
      {
        driveHardStop();
        break;
      }

      pickupAttempts++;
      Serial.printf(">>> PICKUP_RESULT attempt=%d target_try=%d/%d result=%s metal=%d clear_ms=%lu first_clear_ms=%lu\n",
                    pickupAttempts, pickupTries, MAX_PICKUP_TRIES,
                    timedOut ? "CRANE_TIMEOUT" : (confirmed ? "CONFIRMED" : "MISSED_OR_UNCONFIRMED"),
                    metalNow, clearMs, metalLeftAt ? metalLeftAt - pickupCycleStart : 0);
      if (!confirmed && !timedOut && pickupTries < MAX_PICKUP_TRIES && !roundWantsHome())
      {
        pickupStarted = false;
        modeStart = now;
        driveHardStop();
        Serial.println(">>> RETRY same target: remain stopped, repeat pickup");
        break;
      }
      if (confirmed) noteCollected();
      else
      {
        metalLockout = true;
        metalClearSince = 0;
        Serial.println(roundWantsHome()
            ? ">>> pickup unconfirmed: late return takes priority over another retry"
            : ">>> giving up on this target: no count; trigger locked until notch clear");
      }
      if (roundOver() || roundWantsHome())
      {
        driveHardStop();
        setMode(MODE_FORWARD);
        return;
      }
      suppressTargetFor(TARGET_SUPPRESS_MS);
      lastFindOrEvent = now;
      lastTurn = 0; flipCount = 0;
      setMode(MODE_REPOSITION);
      break;
    }

    case MODE_REJECT:
    {
      if (!rejectPivoting)
      {
        // reverse until the encoders say rejectReverseMm (rear guard / time cap end it early)
        float backed = rejectStartMm - odomRawDistanceMM();
        const char *why = nullptr;
        if (backed >= tuning.rejectReverseMm) why = "distance reached";
        else if (rearBlocked())               why = "rear blocked";
        else if (held > REJECT_REVERSE_MAX_MS) why = "time cap";
        if (why)
        {
          Serial.print(">>> REJECT reversed "); Serial.print(backed, 0);
          Serial.print(" mm ("); Serial.print(why); Serial.println(") - pivoting");
          rejectPivoting = true;
          rejectPivotAt = millis();
          rejectStartHdg = imuHeadingDeg();
        }
        break;
      }

      unsigned long pivotHeld = millis() - rejectPivotAt;
      float turned = imuOk() ? fabsf(wrap180(imuHeadingDeg() - rejectStartHdg)) : 0.0f;

      // a DIFFERENT weight: only trusted once the rejected object has swung
      // out of the bottom pair's view (needs the IMU to know how far we've
      // turned), and not on the side the rejected object swings towards
      // (pivoting right, it slides off to the LEFT - weightSide -rejectDir)
      if (wantWeights && weightFound && imuOk() && turned >= REJECT_NEW_WEIGHT_MIN_DEG &&
          weightSide != -rejectDir)
      {
        Serial.print(">>> REJECT pivot cut at "); Serial.print(turned, 0);
        Serial.println(" deg - another weight spotted");
        finishReject();
        startApproach();
        break;
      }

      bool pivotDone = imuOk() ? turned >= tuning.rejectPivotDeg - REJECT_PIVOT_LEAD_DEG
                               : pivotHeld > tuning.repositionTurnMs;
      if (pivotDone || pivotHeld > REJECT_PIVOT_MAX_MS)
      {
        Serial.print(">>> REJECT pivot done: ");
        if (imuOk()) { Serial.print(turned, 0); Serial.println(" deg"); }
        else           Serial.println("timed (no IMU)");
        finishReject();
        suppressTargetFor(REJECT_SUPPRESS_MS);
        setMode(MODE_FORWARD);
      }
      break;
    }

    case MODE_REPOSITION:
      if (held > tuning.repositionTurnMs)
        setMode(MODE_FORWARD);
      break;
  }

  if (goingHome)
  {
    const bool avoiding = mode == MODE_TURN_LEFT || mode == MODE_TURN_RIGHT || mode == MODE_ESCAPE;
    const bool wasPassing = homeDetour.passing();
    const bool wasActive = homeDetour.active();
    homeDetour.update(millis(), avoiding, leftOpen() && rightOpen() &&
                      !sideNearLeft() && !sideNearRight(),
                      poseXmm(), poseYmm(), imuHeadingDeg());
    if (homeDetour.passing() && !wasPassing)
    {
      homeAligning = false;
      headingHoldReset();
      Serial.printf("HOME_DETOUR pass heading=%.1f distance=%.0f\n", homeDetour.heading(), HOME_DETOUR_PASS_MM);
    }
    if (wasActive && !homeDetour.active())
      Serial.println("HOME_DETOUR clear and translated; resume home bearing");
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
      // Obstacle veer and side nudges outrank the home bearing. Large bearing
      // errors use the proven pivot speeds; hysteresis prevents turn chatter.
      if (goingHome && imuOk() && x8Fresh() && steer == 0)
      {
        const float target = homeDetour.passing() ? homeDetour.heading() : homingTargetHeading();
        const float error = wrap180(target - imuHeadingDeg());
        if (homeDetour.passing()) homeAligning = false;
        else if (fabsf(error) > 60) homeAligning = true;
        if (fabsf(error) < 15) homeAligning = false;
        if (homeAligning)
        {
          if (error > 0) turnRight(); else turnLeft();
          break;
        }
        steer = headingHoldSteer(target);
      }
      else if (goingHome) homeAligning = false;
      // Collection keeps its existing heading-hold behaviour.
      else if (USE_HEADING_HOLD && imuOk())
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
      if (NAV_RECOVERY_REVERSE_ENABLED && held < ESCAPE_REV_MS)
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
      int speed = APPROACH_SPEED_PCT;
      const uint16_t viewLeft = weightViewLeftMM(), viewRight = weightViewRightMM();
      if (weightCentreActive)
      {
        steer = 0; // a centre beam gives range, not a left/right error
        approachErrPrev = 0;
      }
      else if (weightSide == 0 && viewLeft > 0 && viewRight > 0)
      {
        float err  = (float)viewLeft - (float)viewRight;
        float dErr = err - approachErrPrev;
        approachErrPrev = err;
        float s = KP_APPROACH * err + KD_APPROACH * dErr;
        if (s >  APPROACH_STEER_MAX) s =  APPROACH_STEER_MAX;
        if (s < -APPROACH_STEER_MAX) s = -APPROACH_STEER_MAX;
        steer = (int)s;
      }
      else
      {
        speed = ONE_SIDE_APPROACH_SPEED_PCT;
        steer = (weightSide < 0) ? -ONE_SIDE_ARC_PCT : +ONE_SIDE_ARC_PCT;
        approachErrPrev = 0;
      }
      drive(speed + steer, speed - steer);
      break;
    }

    case MODE_CREEP:
      drive(CREEP_SPEED_PCT, CREEP_SPEED_PCT);   // straight - nothing to steer on in the blind gap
      break;

    case MODE_PICKUP:
      stopMotors();     // hold still while the crane works
      break;

    case MODE_REJECT:
      // Current round skips the reverse phase and starts a measured pivot.
      if (NAV_RECOVERY_REVERSE_ENABLED && !rejectPivoting)
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
