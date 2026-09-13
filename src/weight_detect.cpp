#include "weight_detect.h"
#include "tof.h"
#include "config.h"

bool     weightFound  = false;
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
    weightSeenSince = 0;
    weightFound     = false;
    weightSide      = 0;
    weightDistMM    = 0;
  }
}
