#ifndef __IR_SENSORS_H
#define __IR_SENSORS_H
// ============================================================================
//  ir_sensors.h  -  2x side analog IR (GP2Y0A41SK), obstacle/wall-scrape
//  nudge. The funnel presence sensor lives in funnel_sensor.h - different
//  job (weight sorting, not drive avoidance) even though it's the same
//  sensor model.
// ============================================================================

#include <Arduino.h>

extern uint16_t irSideLMM, irSideRMM;   // mm, 0 = clear/invalid

void irSensorsInit();
void irSensorsUpdate();

bool sideNearLeft();
bool sideNearRight();

#endif /* __IR_SENSORS_H */
