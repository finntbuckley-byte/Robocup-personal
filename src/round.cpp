#include "round.h"
#include "config.h"
#include "funnel_sensor.h"

// ============================================================================
//  round.cpp
//
//  Start: GO button (PIN_GO, debounced) or, while PIN_GO is -1, automatically
//  AUTO_START_DELAY_MS after roundInit(). The rules have the robot switched
//  on by hand at the start, so an auto-start delay is legal - but a button is
//  better (start heading gets zeroed at that moment once the IMU is back).
//
//  End: ROUND_MS minus ROUND_END_MARGIN_MS, so the robot is already still
//  when the round is called and nothing is mid-motion at scoring.
//
//  Targets on board = weights the funnel classified as metal, minus any
//  delivered. ASSUMPTION: every metal-classified weight stays on board
//  (dummies are rejected by the gate/flap). TODO(verify) once the gate
//  sorting logic exists.
// ============================================================================

enum RoundPhase { PHASE_WAITING, PHASE_RUNNING, PHASE_OVER };
static RoundPhase phase = PHASE_WAITING;

static unsigned long initAt = 0;
static unsigned long startAt = 0;
static bool justStarted = false;
static int deliveredCount = 0;

static unsigned long goDownSince = 0;

static bool goPressed()
{
  if (PIN_GO < 0) return false;
  bool active = GO_ACTIVE_LOW ? (digitalRead(PIN_GO) == LOW) : (digitalRead(PIN_GO) == HIGH);
  if (!active) { goDownSince = 0; return false; }
  if (goDownSince == 0) goDownSince = millis();
  return millis() - goDownSince >= GO_DEBOUNCE_MS;
}

void roundInit()
{
  if (PIN_GO >= 0) pinMode(PIN_GO, GO_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  initAt = millis();
  phase = PHASE_WAITING;
  Serial.print("Round: waiting for ");
  if (PIN_GO >= 0) Serial.println("GO button");
  else { Serial.print("auto-start in "); Serial.print(AUTO_START_DELAY_MS); Serial.println("ms"); }
}

void roundUpdate()
{
  justStarted = false;

  switch (phase)
  {
    case PHASE_WAITING:
    {
      bool go = (PIN_GO >= 0) ? goPressed() : (millis() - initAt >= AUTO_START_DELAY_MS);
      if (go)
      {
        startAt = millis();
        justStarted = true;
        phase = PHASE_RUNNING;
        Serial.println(">>> ROUND START");
      }
      break;
    }

    case PHASE_RUNNING:
      if (roundElapsedMs() >= ROUND_MS - ROUND_END_MARGIN_MS)
      {
        phase = PHASE_OVER;
        Serial.print(">>> ROUND OVER  real:"); Serial.print(realWeightCount);
        Serial.print(" dummy:"); Serial.print(dummyCount);
        Serial.print(" onboard:"); Serial.println(targetsOnBoard());
      }
      break;

    case PHASE_OVER:
      break;
  }
}

bool roundWaiting()     { return phase == PHASE_WAITING; }
bool roundRunning()     { return phase == PHASE_RUNNING; }
bool roundOver()        { return phase == PHASE_OVER; }
bool roundJustStarted() { return justStarted; }

unsigned long roundElapsedMs()
{
  return (phase == PHASE_WAITING) ? 0 : millis() - startAt;
}

int targetsOnBoard()
{
  int n = realWeightCount - deliveredCount;
  return n < 0 ? 0 : n;
}

void noteDelivered(int n) { deliveredCount += n; }

bool roundWantsHome()
{
  if (targetsOnBoard() >= MAX_TARGETS_ON_BOARD) return true;
#if USE_HOMING
  return roundElapsedMs() >= RETURN_HOME_AT_MS;
#else
  return false;    // no homing yet - keep collecting to the cap all round
#endif
}

bool roundWantsWeights() { return roundRunning() && !roundWantsHome(); }

const char* roundPhaseName()
{
  switch (phase)
  {
    case PHASE_WAITING: return "WAIT";
    case PHASE_RUNNING: return roundWantsHome() ? "HOME" : "RUN";
    default:            return "OVER";
  }
}
