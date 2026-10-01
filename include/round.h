#ifndef __ROUND_H
#define __ROUND_H
// ============================================================================
//  round.h  -  the 2-minute competition round: start trigger (GO button, or
//  auto-start while it isn't wired), round timer, and the on-board target
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
void noteDelivered(int n);        // for the (future) DELIVER state

// false once the cap is reached or it's late in the round - navigation
// stops approaching/picking up new weights
bool roundWantsWeights();
// true late in the round or at the cap - where RETURN_HOME would kick in
bool roundWantsHome();

const char* roundPhaseName();

#endif /* __ROUND_H */
