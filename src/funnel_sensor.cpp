#include "funnel_sensor.h"
#include "config.h"
#include <math.h>

// ============================================================================
//  funnel_sensor.cpp
//
//  State flow (simple edge-triggered classifier, decoupled from
//  collection.cpp's pickup FSM - see funnel_sensor.h for why):
//
//    CLEAR    funnel IR sees nothing.
//    PRESENT  funnel IR just tripped. Start a classification window
//             (SORT_CLASSIFY_WINDOW_MS) and watch the inductive sensor for
//             a trigger anywhere in that window (metal can register partway
//             through the slide, not necessarily at the exact instant the
//             IR trips).
//    VERDICT  window elapsed -> count it as real (metal seen) or dummy (no
//             metal seen), print the result, then wait for the IR to clear
//             before arming again - so one object isn't counted twice while
//             it's still sitting there.
//
//  INDUCTIVE WIRING NOTE: most inductive proximity sensors (e.g. the
//  LJ18A3-8-Z/BY used in this course) are NPN and pull LOW when they detect
//  metal. INDUCTIVE_ACTIVE_LOW in config.h defaults to that - flip it if
//  yours reads the other way.
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

static uint16_t funnelMM = 0;
static unsigned long presentSince = 0;
static bool sawInductiveThisPass = false;

static unsigned long irLastRead = 0;

static float irReadVolts(int pin)
{
  long sum = 0;
  for (int i = 0; i < IR_SAMPLES; i++) sum += analogRead(pin);
  float counts = (float)sum / IR_SAMPLES;
  return counts * (IR_ADC_VREF / IR_ADC_COUNTS);
}

static uint16_t irVoltsToMM(float volts, float A, float B, int minMM, int maxMM)
{
  if (volts < 0.10f) return 0;
  float mm = A * pow(volts, B);
  if (mm < minMM || mm > maxMM) return 0;
  return (uint16_t)mm;
}

static inline bool inductiveTriggered()
{
  int lvl = digitalRead(PIN_INDUCTIVE);
  return INDUCTIVE_ACTIVE_LOW ? (lvl == LOW) : (lvl == HIGH);
}

void funnelSensorInit()
{
#if USE_FUNNEL_SORT
  pinMode(PIN_IR_FUNNEL, INPUT);
  pinMode(PIN_INDUCTIVE, INPUT);
#endif
}

bool weightInFunnel() { return funnelMM > 0 && funnelMM < FUNNEL_PRESENT_MM; }

void funnelSortUpdate()
{
#if USE_FUNNEL_SORT
  if (millis() - irLastRead >= IR_READ_MS)
  {
    irLastRead = millis();
    funnelMM = irVoltsToMM(irReadVolts(PIN_IR_FUNNEL), IR_SIDE_A, IR_SIDE_B,
                           IR_SIDE_MIN_MM, IR_SIDE_MAX_MM);
  }

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
          Serial.print(">>> FUNNEL: non-metal (dummy/Sphero) #"); Serial.println(dummyCount);
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
