#include "weight_detect.h"
#include "tof.h"
#include "x8.h"
#include "config.h"

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
  return x8Fresh() && tofOk(TOF_BL) && pairSeesWeight(tofBL, x8LeftMM());
}
static bool weightRight()
{
  return x8Fresh() && tofOk(TOF_BR) && pairSeesWeight(tofBR, x8RightMM());
}

bool     weightFound  = false;
int      weightSide   = 0;
uint16_t weightDistMM = 0;
int      pickupAttempts = 0;

static unsigned long weightSeenSince = 0;
static unsigned long weightLastSeen = 0;   // last update the candidate was actually seen (grace)
static unsigned long targetSuppressUntil = 0;

void suppressTargetFor(unsigned long ms)
{
  targetSuppressUntil = millis() + ms;
  weightSeenSince = 0;
  weightFound = false;
  weightSide = 0;
  weightDistMM = 0;
}

bool targetSuppressed() { return millis() < targetSuppressUntil; }

void weightDetectUpdate()
{
  bool L = weightLeft();
  bool R = weightRight();

  if (targetSuppressed()) { L = R = false; }

  if (L || R)
  {
    if (weightSeenSince == 0) weightSeenSince = millis();

    if (millis() - weightSeenSince >= WEIGHT_STICK_MS)
    {
      weightFound = true;
      weightLastSeen = millis();
      if (L && R)
      {
        weightSide   = 0;
        weightDistMM = (tofBL < tofBR) ? tofBL : tofBR;
      }
      else if (L) { weightSide = -1; weightDistMM = tofBL; }
      else        { weightSide = +1; weightDistMM = tofBR; }
    }
  }
  else
  {
    // Grace: one dropped reading mustn't end an approach (arena 28/9: the
    // bottom ToF read 0 for a single frame at 320-455mm three times and the
    // robot gave up on real weights). Keep the last candidate - and its
    // persistence timer - for WEIGHT_LOST_MS before dropping it.
    if (weightFound && millis() - weightLastSeen < WEIGHT_LOST_MS) return;
    weightSeenSince = 0;
    weightFound     = false;
    weightSide      = 0;
    weightDistMM    = 0;
  }
}
