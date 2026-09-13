#ifndef __FUNNEL_SENSOR_H
#define __FUNNEL_SENSOR_H
// ============================================================================
//  funnel_sensor.h  -  weight sorting at the end of the funnel.
//
//  Two sensors, one job: figure out what just went past the funnel end.
//    - analog IR (PIN_IR_FUNNEL)   : "is something here at all"
//    - inductive sensor (PIN_INDUCTIVE) : "is it metal"
//
//  This runs INDEPENDENTLY of the collection/navigation state machines -
//  it just watches the funnel end continuously and classifies whatever
//  passes, regardless of what put it there. That's deliberate: it's the
//  simplest correct thing, and it stays correct even if you later add
//  another way for a weight to reach the funnel.
//
//  If you want it tied more tightly to a specific pickup cycle (e.g. "only
//  arm classification a few hundred ms after collection_start()"), that's a
//  small addition in funnel_sensor.cpp - see the comment above
//  funnelSortUpdate().
// ============================================================================

#include <Arduino.h>

void funnelSensorInit();
void funnelSortUpdate();

bool weightInFunnel();          // true while something is currently detected

// classification counters/telemetry - read-only from outside this module
extern int realWeightCount;     // classified metal
extern int dummyCount;          // classified non-metal (dummy or Sphero)

#endif /* __FUNNEL_SENSOR_H */
