#ifndef __WEIGHT_DETECT_H
#define __WEIGHT_DETECT_H
// ============================================================================
//  weight_detect.h  -  turns the bottom-ToF-vs-8x8 differential into a
//  stable candidate to drive at. See config.h for thresholds.
//
//  KNOWN (and intended) LIMITATION: a dummy and a real weight give the same
//  ToF signature (something short and close) - this module can't tell them
//  apart. That's the funnel_sensor module's job, AFTER pickup - this one
//  only decides what's worth driving at and picking up.
// ============================================================================

#include <Arduino.h>

extern bool     weightFound;
extern int      weightSide;      // -1 left, +1 right, 0 centred/both
extern uint16_t weightDistMM;
extern bool weightCentreActive; // confirmed centre-facing top-ToF candidate
extern int      pickupAttempts;  // telemetry: how many times PICKUP has run

void suppressTargetFor(unsigned long ms);
bool targetSuppressed();
void weightDetectUpdate();
// Fresh ranges by viewing side; raw tofBL/tofBR remain physical telemetry.
uint16_t weightViewLeftMM();
uint16_t weightViewRightMM();
uint16_t weightViewCentreMM();

#endif /* __WEIGHT_DETECT_H */
