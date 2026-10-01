#include "weight_detect.h"
#include "tof.h"
#include "x8.h"
#include "config.h"
#include "sensor_geometry.h"

static uint16_t viewRange(bool left)
{
  const int index = lowerViewIndex(left, LOWER_BEAMS_CROSSED, TOF_BL, TOF_BR);
  return tofOk(index) ? tofMM[index] : 0;
}
uint16_t weightViewLeftMM() { return viewRange(true); }
uint16_t weightViewRightMM() { return viewRange(false); }
uint16_t weightViewCentreMM() { return tofOk(TOF_TOP) ? tofTop : 0; }

// A bottom ToF sees something, and the 8x8's obstacle band on the same side
// sees nothing at a similar distance -> something short -> weight candidate.
// (Replaces the old bottom-vs-top-front ToF pairing.)
static bool pairSeesWeight(uint16_t bot, uint16_t top)
{
  if (bot == 0 || bot < WEIGHT_MIN_MM || bot > WEIGHT_MAX_MM) return false;
  if (top == 0) return true;
  return top >= bot + DIFF_CLEAR_MARGIN_MM;
}

// Without a fresh 8x8 frame we can't tell a weight from a wall, so no
// candidates - better to miss one than to drive into a wall "approaching" it.
static bool weightLeft()
{
  return x8Fresh() && pairSeesWeight(weightViewLeftMM(), x8LeftMM());
}
static bool weightRight()
{
  return x8Fresh() && pairSeesWeight(weightViewRightMM(), x8RightMM());
}

bool     weightFound  = false;
bool     weightCentreActive = false;
int      weightSide   = 0;
uint16_t weightDistMM = 0;
int      pickupAttempts = 0;

static unsigned long weightSeenSince = 0;
static unsigned long targetSuppressUntil = 0;

void suppressTargetFor(unsigned long ms)
{
  targetSuppressUntil = millis() + ms;
  weightSeenSince = 0;
  weightFound = false;
  weightCentreActive = false;
  weightSide = 0;
  weightDistMM = 0;
}

bool targetSuppressed() { return millis() < targetSuppressUntil; }

void weightDetectUpdate()
{
  bool L = weightLeft();
  bool R = weightRight();

  // Centre-facing top beam uses the same range, debounce and obstacle veto.
  bool C = x8Fresh() && pairSeesWeight(weightViewCentreMM(), x8FrontMM());
  if (targetSuppressed()) { L = R = C = false; }
  weightCentreActive = false;

  if (L || R || C)
  {
    if (weightSeenSince == 0) weightSeenSince = millis();

    if (millis() - weightSeenSince >= WEIGHT_STICK_MS)
    {
      weightFound = true;
      if (C)
      {
        weightCentreActive = true;
        weightSide = 0;
        weightDistMM = weightViewCentreMM();
      }
      else if (L && R)
      {
        weightSide   = 0;
        weightDistMM = (weightViewLeftMM() < weightViewRightMM()) ? weightViewLeftMM() : weightViewRightMM();
      }
      else if (L) { weightSide = -1; weightDistMM = weightViewLeftMM(); }
      else        { weightSide = +1; weightDistMM = weightViewRightMM(); }
    }
  }
  else
  {
    weightSeenSince = 0;
    weightFound     = false;
    weightSide      = 0;
    weightDistMM    = 0;
  }
}
