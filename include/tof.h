#ifndef __TOF_H
#define __TOF_H
// ============================================================================
//  tof.h  -  7x ToF: 2 low (weight), 4 top+corner (obstacle), 1 rear (obstacle)
//  See config.h for the full sensor map and the TOF_* index constants.
//  Convention: distance 0 == "nothing in range / clear".
// ============================================================================

#include <Arduino.h>

extern uint16_t tofMM[];   // indexed by TOF_BL..TOF_REAR, see config.h

// friendly references - same identifiers used throughout navigation/weight_detect
extern uint16_t &tofBL, &tofBR, &tofFL, &tofFR, &cornerL, &cornerR, &tofRear;

void tofInit();
void tofUpdate();

uint16_t frontWallMM();
bool weightLeft();
bool weightRight();
bool cornerNearLeft();
bool cornerNearRight();
bool rearBlocked();

#endif /* __TOF_H */
