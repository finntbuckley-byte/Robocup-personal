#include "round.h"
#include "config.h"
#include "funnel_sensor.h"
#include "colour.h"

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
//  Targets on board = successful pickups (navigation calls noteCollected()
//  when a crane cycle started on metal AND the metal is gone from the notch
//  afterwards), minus any delivered. Dummies are never picked up - pickup is
//  gated on the inductive sensor.
// ============================================================================

enum RoundPhase { PHASE_WAITING, PHASE_RUNNING, PHASE_OVER };
static RoundPhase phase = PHASE_WAITING;

static unsigned long initAt = 0;
static unsigned long startAt = 0;
static bool justStarted = false;
static int deliveredCount = 0;
static int collectedCount = 0;
static bool lateReturnConsumed = false;
void roundAcknowledgeReturn() {
  if (roundElapsedMs() >= RETURN_HOME_AT_MS) lateReturnConsumed = true;
}

static unsigned long goDownSince = 0;
static unsigned long goUpSince = 0;
static bool goArmed = false;     // GO counts only after it's been seen RELEASED

static bool goActiveRaw()
{
  return GO_ACTIVE_LOW ? (digitalRead(PIN_GO) == LOW) : (digitalRead(PIN_GO) == HIGH);
}

// Safety: a button held, stuck, or misread at boot must never start the
// round. GO arms only after GO_DEBOUNCE_MS of continuous "released", then
// needs GO_DEBOUNCE_MS of continuous "pressed".
static bool goPressed()
{
  if (PIN_GO < 0) return false;
  bool active = goActiveRaw();

  if (!goArmed)
  {
    if (active) { goUpSince = 0; return false; }
    if (goUpSince == 0) goUpSince = millis();
    if (millis() - goUpSince >= GO_DEBOUNCE_MS)
    {
      goArmed = true;
      Serial.println("Round: GO armed - press to start");
    }
    return false;
  }

  if (!active) { goDownSince = 0; return false; }
  if (goDownSince == 0) goDownSince = millis();
  return millis() - goDownSince >= GO_DEBOUNCE_MS;
}

void roundInit()
{
  collectedCount=deliveredCount=0;lateReturnConsumed=false;
  if (PIN_GO >= 0) pinMode(PIN_GO, GO_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  initAt = millis();
  phase = PHASE_WAITING;
  goArmed = false;
  goUpSince = goDownSince = 0;
  Serial.print("Round: waiting for ");
  if (PIN_GO >= 0)
  {
    delay(5);   // let the pull-up settle before the diagnostic read
    Serial.print("GO button (pin "); Serial.print(PIN_GO);
    Serial.print(" reads "); Serial.print(digitalRead(PIN_GO) ? "HIGH" : "LOW");
    Serial.print(" = "); Serial.print(goActiveRaw() ? "PRESSED" : "released");
    Serial.println(")");
  }
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
        goArmed = false;               // GO must be released before a stop press counts
        goUpSince = goDownSince = 0;
        Serial.println(">>> ROUND START");
      }
      break;
    }

    case PHASE_RUNNING:
      if (GO_STOPS_ROUND && goPressed())       // testing only - see config.h
      {
        phase = PHASE_OVER;
        Serial.print(">>> ROUND STOPPED (GO) at "); Serial.print(roundElapsedMs()); Serial.println(" ms");
      }
      else if (roundElapsedMs() >= ROUND_MS - ROUND_END_MARGIN_MS)
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
  int n = collectedCount - deliveredCount;
  return n < 0 ? 0 : n;
}

void noteCollected()
{
  collectedCount++;
  Serial.print(">>> TARGET COLLECTED - on board: "); Serial.println(targetsOnBoard());
  if (targetsOnBoard() >= MAX_TARGETS_ON_BOARD)
    Serial.println(">>> THREE COLLECTED: return home with third held");
}

void noteDelivered(int n) {
  if(n<0) return;
  if(n>targetsOnBoard()) n=targetsOnBoard();
  deliveredCount += n;
  roundAcknowledgeReturn();
  Serial.println(">>> TIMED UNLOAD COMPLETE: load assumed delivered, no exit sensor");
}

bool roundWantsHome()
{
  if (targetsOnBoard() >= MAX_TARGETS_ON_BOARD) return true;
#if USE_HOMING
  return !lateReturnConsumed && roundElapsedMs() >= RETURN_HOME_AT_MS;
#else
  return false;    // no homing yet - keep collecting to the cap all round
#endif
}

bool roundWantsWeights() {
  return roundRunning() && !roundWantsHome() &&
    roundElapsedMs()+PICKUP_TIMEOUT_MS+PICKUP_VERIFY_TIMEOUT_MS+500 < ROUND_MS-ROUND_END_MARGIN_MS &&
    colourOk() && colourSurface()==COLOUR_FLOOR;
}

const char* roundPhaseName()
{
  switch (phase)
  {
    case PHASE_WAITING: return "WAIT";
    case PHASE_RUNNING: return roundWantsHome() ? "HOME" : "RUN";
    default:            return "OVER";
  }
}
