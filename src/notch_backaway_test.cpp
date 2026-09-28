#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "drive.h"
#include "motor.h"
#include "tof.h"
#include "x8.h"

// Standalone bench test for the notch-ToF / inductive back-away condition.
// Motors stay neutral until 'a' is entered; 'x' hard-stops and disarms.
enum TestState { TEST_DISARMED, TEST_ARMED, TEST_REVERSE, TEST_TURN, TEST_WAIT_CLEAR };
static TestState state = TEST_DISARMED;
static unsigned long stateAt = 0;
static unsigned long settledSince = 0;
static int settleAnchor = -1;   // reading at the start of the current settle window
static unsigned long metalAbsentSince = 0;
static unsigned long lastPrint = 0;
static bool backawayLatched = false;
static int turnDir = -1;

static const char *stateName()
{
  switch (state)
  {
    case TEST_DISARMED:   return "DISARMED";
    case TEST_ARMED:      return "ARMED";
    case TEST_REVERSE:    return "REVERSE";
    case TEST_TURN:       return "TURN";
    case TEST_WAIT_CLEAR: return "WAIT_CLEAR";
    default:              return "?";
  }
}

static bool inductiveMetal()
{
  int level = digitalRead(PIN_INDUCTIVE);
  return INDUCTIVE_ACTIVE_LOW ? level == LOW : level == HIGH;
}

static int roomierSide()
{
  if (!x8Fresh()) return -1;
  uint16_t left = x8LeftMM(), right = x8RightMM();
  uint16_t leftRoom = left == 0 ? 0xFFFF : left;
  uint16_t rightRoom = right == 0 ? 0xFFFF : right;
  return rightRoom >= leftRoom ? +1 : -1;
}

static void hardStopAndDisarm()
{
  driveHardStop();
  state = TEST_DISARMED;
  settledSince = 0;
  settleAnchor = -1;
  Serial.println("DISARMED - motors hard-stopped");
}

static void printStatus()
{
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  Serial.print(millis()); Serial.print('\t');
  Serial.print(stateName()); Serial.print('\t');
  Serial.print("upright=");
  if (!tofOk(TOF_UPRIGHT)) Serial.print("STALE"); else Serial.print(tofUpright);
  Serial.print('\t');
  Serial.print("metal="); Serial.print(inductiveMetal() ? 1 : 0); Serial.print('\t');
  Serial.print("settledMs=");
  Serial.print(settledSince == 0 ? 0 : (long)(millis() - settledSince));
  Serial.print('\t');
  Serial.print("metalAbsentMs=");
  Serial.print(metalAbsentSince == 0 ? 0 : (long)(millis() - metalAbsentSince));
  Serial.print('\t');
  Serial.print("rear=");
  if (!tofOk(TOF_REAR)) Serial.print("STALE"); else Serial.print(tofRear);
  Serial.print('\t');
  Serial.print("x8L/R=");
  if (!x8Fresh()) Serial.print("STALE");
  else { Serial.print(x8LeftMM()); Serial.print('/'); Serial.print(x8RightMM()); }
  Serial.print('\t');
  Serial.print("drive="); Serial.print(lastDriveLeftPct()); Serial.print('/');
  Serial.println(lastDriveRightPct());
}

void setup()
{
  Serial.begin(115200);
  unsigned long waitAt = millis();
  while (!Serial && millis() - waitAt < 3000) {}

  motor_init();
  driveHardStop();
  pinMode(PIN_INDUCTIVE, INPUT);
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);

  Serial.println("\n--- notch back-away bench test ---");
  Serial.println("Motors remain stopped until armed. Initialising ToFs and SEN0628...");
  tofInit();
  x8Init();
  Serial.println("Commands: a=arm, x=hard stop/disarm, ?=help");
  Serial.println("Trigger: upright ToF 30-80 mm AND settled (<=4 mm drift) for 400 ms AND no");
  Serial.println("inductive metal by then. A weight still sliding closer never settles, so it");
  Serial.println("should not trigger until it stops - watch settledMs in the telemetry line.");
  Serial.println("Response: reverse 500 ms (rear guard), turn 700 ms, then wait for >90 mm.");
  Serial.println("Test-only behavior: it does not resume forward drive after backing away.");
}

void loop()
{
  tofUpdate();
  x8Update();

  if (Serial.available())
  {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'a')
    {
      driveHardStop();
      settledSince = 0;
      settleAnchor = -1;
      backawayLatched = false;
      state = TEST_ARMED;
      Serial.println("ARMED - place/hold test object in the notch band");
    }
    else if (c == 'x') hardStopAndDisarm();
    else if (c == '?')
    {
      Serial.println("a=arm the sensor-triggered test; x=hard stop and disarm");
      Serial.println("Keep robot on blocks. It remains stopped after each response.");
    }
  }

  bool metal = inductiveMetal();

  // Debounce the "no metal" side the same way the real pickup trigger
  // debounces the "yes metal" side (metalConfirmed() in navigation.cpp): a
  // single noisy LOW sample must not be trusted as "confirmed absent" while
  // the sensor has otherwise been reading metal steadily.
  if (metal) metalAbsentSince = 0;
  else if (metalAbsentSince == 0) metalAbsentSince = millis();
  bool metalConfirmedAbsent = !metal &&
      (millis() - metalAbsentSince >= UPRIGHT_BACKAWAY_METAL_ABSENT_MS);

  switch (state)
  {
    case TEST_ARMED:
    {
      if (tofOk(TOF_UPRIGHT) && tofUpright >= UPRIGHT_BACKAWAY_CLEAR_MM)
      {
        backawayLatched = false;
        settledSince = 0;
        settleAnchor = -1;
      }

      bool inBand = tofOk(TOF_UPRIGHT) &&
                    tofUpright >= UPRIGHT_BACKAWAY_MIN_MM &&
                    tofUpright <= UPRIGHT_BACKAWAY_MAX_MM;

      if (!inBand)
      {
        settledSince = 0;
        settleAnchor = -1;
      }
      else
      {
        // Settled = the reading has stayed within tolerance of the value it
        // had when the settle window opened (settleAnchor), for the whole
        // window - not just frame-to-frame. A slow, steady creep (a weight
        // sliding in) drifts well past the tolerance over the window even
        // though each individual sample-to-sample step is small, so it keeps
        // resetting the anchor and never settles. A resting dummy/lying
        // object stops moving and settles almost immediately.
        // Bench-confirmed 2026-09-28: frame-to-frame-only tolerance let a
        // slowly-approaching real weight fire the trigger ~400ms before the
        // inductive sensor caught up.
        int delta = (settleAnchor < 0) ? 0 : abs(tofUpright - settleAnchor);
        if (settleAnchor < 0 || delta > UPRIGHT_BACKAWAY_SETTLE_TOL_MM)
        {
          settledSince = millis();
          settleAnchor = tofUpright;
        }

        // "settled" fires the instant SETTLE_MS of low-drift readings elapse;
        // CONFIRM_MS is extra debounce measured from that moment, not from
        // band entry, so total trigger time = SETTLE_MS + CONFIRM_MS.
        unsigned long settledAt = settledSince + UPRIGHT_BACKAWAY_SETTLE_MS;
        bool settled = millis() >= settledAt;

        if (settled && metalConfirmedAbsent && !backawayLatched &&
            millis() - settledAt >= UPRIGHT_BACKAWAY_CONFIRM_MS)
        {
          backawayLatched = true;
          turnDir = roomierSide();
          state = TEST_REVERSE;
          stateAt = millis();
          Serial.print("TRIGGER: settled, no inductive metal (confirmed absent ");
          Serial.print(UPRIGHT_BACKAWAY_METAL_ABSENT_MS); Serial.print("ms), upright ToF=");
          Serial.print(tofUpright); Serial.print(" mm; turnDir=");
          Serial.println(turnDir > 0 ? "right" : "left");
        }
      }
      driveHardStop();
      break;
    }

    case TEST_REVERSE:
      if (millis() - stateAt < REJECT_REVERSE_MS)
      {
        if (rearBlocked()) driveHardStop();
        else drive(-REJECT_REVERSE_PCT, -REJECT_REVERSE_PCT);
      }
      else
      {
        state = TEST_TURN;
        stateAt = millis();
        driveHardStop();
      }
      break;

    case TEST_TURN:
      if (millis() - stateAt < REPOSITION_TURN_MS)
        drive(turnDir * REPOSITION_SPEED_PCT, -turnDir * REPOSITION_SPEED_PCT);
      else
      {
        driveHardStop();
        state = TEST_WAIT_CLEAR;
        Serial.println("Response complete - stopped; clear the notch past 90 mm to re-arm");
      }
      break;

    case TEST_WAIT_CLEAR:
      driveHardStop();
      if (tofOk(TOF_UPRIGHT) && tofUpright >= UPRIGHT_BACKAWAY_CLEAR_MM)
      {
        backawayLatched = false;
        settledSince = 0;
        settleAnchor = -1;
        state = TEST_ARMED;
        Serial.println("Notch clear - ARMED again");
      }
      break;

    case TEST_DISARMED:
      driveHardStop();
      break;
  }

  // Same rear-clearance condition as navigation; hard stop overrides any reverse.
  if (isReversing() && rearBlocked()) driveHardStop();
  printStatus();
}
