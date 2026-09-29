#ifndef __NAVIGATION_H
#define __NAVIGATION_H
// ============================================================================
//  navigation.h  -  nav + weight-seek + real pickup state machine.
//  Only call navigationUpdate() while roundRunning() (see nav_main.cpp).
// ============================================================================

#include <Arduino.h>

void navigationInit();
void navigationUpdate();
const char* modeName();
int rejectedCount();      // dummies/non-metal backed away from (telemetry)
int approachGiveUpCount(); // approaches dropped for not closing in / stalled (telemetry)

// Live-tunable copy of the REJECT_REVERSE_MM/REJECT_PIVOT_DEG/
// REPOSITION_TURN_MS/LYING_DISCREPANCY_MM/LYING_CONFIRM_MS/CREEP_HARD_CAP_MS
// constants (config.h), starting from those values. navigation.cpp reads from
// this, not the raw consts, so nav_main.cpp's serial menu (':rev'/':pdeg'/
// ':turn'/':disc'/':lconf'/':ccap') can adjust them
// live during a round simulation - no reflash needed while testing. Once a
// setting works, copy it back into config.h by hand.
struct NavTuning
{
    int           rejectReverseMm;   // REJECT reverse distance, by encoder (REJECT_REVERSE_MM)
    int           rejectPivotDeg;    // REJECT pivot angle, by IMU (REJECT_PIVOT_DEG)
    unsigned long repositionTurnMs;  // REPOSITION pivot time (+ REJECT pivot if the IMU is down)
    int           lyingDiscrepancyMm;
    unsigned long lyingConfirmMs;
    unsigned long creepCapMs;      // CREEP give-up once the notch has seen something (CREEP_HARD_CAP_MS)
};
NavTuning &navTuning();   // mutable reference - edit fields directly to tune live

#endif /* __NAVIGATION_H */
