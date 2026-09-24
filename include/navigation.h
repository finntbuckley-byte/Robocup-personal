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

#endif /* __NAVIGATION_H */
