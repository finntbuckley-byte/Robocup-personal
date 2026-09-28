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

#endif /* __NAVIGATION_H */
