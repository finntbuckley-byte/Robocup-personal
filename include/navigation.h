#ifndef __NAVIGATION_H
#define __NAVIGATION_H
// ============================================================================
//  navigation.h  -  full nav + weight-seek + real pickup state machine.
// ============================================================================

#include <Arduino.h>

void navigationInit();
void navigationUpdate();
const char* modeName();

#endif /* __NAVIGATION_H */
