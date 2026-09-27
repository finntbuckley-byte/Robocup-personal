#include "funnel_sensor.h"
#include "config.h"
#include "tof.h"

// ============================================================================
//  funnel_sensor.cpp
//
//  State flow (simple edge-triggered classifier, decoupled from
//  collection.cpp's pickup FSM - see funnel_sensor.h for why):
//
//    CLEAR    weight-detect ToF sees nothing in the funnel band.
//    PRESENT  something just entered the band. Start a classification window
//             (SORT_CLASSIFY_WINDOW_MS) and watch the inductive sensor for
//             a trigger anywhere in that window (metal can register partway
//             through the slide, not necessarily at the exact instant
//             presence trips).
//    VERDICT  window elapsed -> count it as real (metal seen) or dummy (no
//             metal seen), print the result, then wait for the funnel to
//             clear before arming again - so one object isn't counted twice
//             while it's still sitting there.
//
//  Presence comes from the weight-detect ToF across the notch (TOF_UPRIGHT,
//  CON27) - tof.cpp keeps it fresh; this module only interprets it.
//
//  INDUCTIVE WIRING: LJ18A3-8-Z/BY -> inductive level-shift board -> CON70
//  (A6Z, pin 20). NPN sensors pull LOW on metal; INDUCTIVE_ACTIVE_LOW in
//  config.h matches the partner's original tested METALLIC = 0.
//
//  TO TIE THIS TO A SPECIFIC PICKUP CYCLE instead of running continuously:
//  add an "armed" flag here, set true a fixed delay after collection_start()
//  (once you've measured how long the crane takes to reach the drop point),
//  and only evaluate funnelSortUpdate()'s state machine while armed.
// ============================================================================

int realWeightCount = 0;
int dummyCount       = 0;

enum FunnelState { FUNNEL_CLEAR, FUNNEL_PRESENT, FUNNEL_VERDICT_WAIT };
static FunnelState state = FUNNEL_CLEAR;

static unsigned long presentSince = 0;
static bool sawInductiveThisPass = false;

static inline bool inductiveTriggered()
{
  int lvl = digitalRead(PIN_INDUCTIVE);
  return INDUCTIVE_ACTIVE_LOW ? (lvl == LOW) : (lvl == HIGH);
}

void funnelSensorInit()
{
#if USE_FUNNEL_SORT
  pinMode(PIN_INDUCTIVE, INPUT);
#endif
}

bool weightInFunnel()
{
  return tofOk(TOF_UPRIGHT) &&
         tofUpright >= FUNNEL_MIN_MM && tofUpright < FUNNEL_PRESENT_MM;
}

bool inductiveMetalNow() { return inductiveTriggered(); }

void funnelSortUpdate()
{
#if USE_FUNNEL_SORT
  bool present = weightInFunnel();

  switch (state)
  {
    case FUNNEL_CLEAR:
      if (present)
      {
        presentSince = millis();
        sawInductiveThisPass = inductiveTriggered();
        state = FUNNEL_PRESENT;
      }
      break;

    case FUNNEL_PRESENT:
      if (inductiveTriggered()) sawInductiveThisPass = true;

      if (millis() - presentSince >= SORT_CLASSIFY_WINDOW_MS)
      {
        if (sawInductiveThisPass)
        {
          realWeightCount++;
          Serial.print(">>> FUNNEL: metal (real weight) #"); Serial.println(realWeightCount);
        }
        else
        {
          dummyCount++;
          Serial.print(">>> FUNNEL: non-metal (dummy) #"); Serial.println(dummyCount);
        }
        state = FUNNEL_VERDICT_WAIT;
      }
      break;

    case FUNNEL_VERDICT_WAIT:
      if (!present) state = FUNNEL_CLEAR;   // don't recount the same object
      break;
  }
#endif
}
