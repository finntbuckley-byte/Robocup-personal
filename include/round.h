#ifndef __ROUND_H
#define __ROUND_H
// ============================================================================
//  round.h  -  timed competition or unlimited arena run: GO start/latched stop,
//  automatic start only while the button isn't wired, round timer, and the on-board target
//  cap from the rules (max 3 targets on board at the end).
//
//  WAITING -> RUNNING -> OVER. Motors must be stopped in WAITING and OVER -
//  nav_main.cpp enforces that; navigation only runs while RUNNING.
// ============================================================================

#include <Arduino.h>

void roundInit();
void roundUpdate();

bool roundWaiting();
bool roundRunning();
bool roundOver();
bool roundJustStarted();          // true for exactly one roundUpdate() tick

unsigned long roundElapsedMs();   // 0 until started

int  targetsOnBoard();            // successful pickups not yet delivered
void noteCollected();             // third requests return, retaining magnet hold
void roundAcknowledgeReturn();    // consume the late-round trigger once
void noteDelivered(int n);        // after unloading and confirmed gate closure

// Requires floor colour and no return request; timed mode also reserves pickup time.
bool roundWantsWeights();
// true with a load: at capacity, on own home colour, or the timed return trigger
bool roundWantsHome();

const char* roundPhaseName();

#endif /* __ROUND_H */
