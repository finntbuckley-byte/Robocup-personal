#ifndef __TOF_H
#define __TOF_H
// ============================================================================
//  tof.h  -  5x ToF on the XSHUT chain:
//    bottom-left / bottom-right (VL53L0X) - weight detection, APPROACH
//    weight-detect across the notch       - lying-weight reject "bottom"
//    rear (VL53L1X)                       - reversing clearance
//    baseplate-top (VL53L0X)              - lying-weight reject "top"
//                                            (proposed, not yet fitted)
//  See config.h for the sensor map and the TOF_* index constants.
//  The front obstacle sensor (SEN0628 8x8) is in x8.h, not here.
//  Convention: distance 0 == "nothing in range / clear / sensor missing".
// ============================================================================

#include <Arduino.h>

extern uint16_t tofMM[];   // indexed by TOF_BL..TOF_TOP, see config.h

// friendly references - same identifiers used throughout navigation/weight_detect
extern uint16_t &tofBL, &tofBR, &tofUpright, &tofRear, &tofTop;

void tofInit();
void tofUpdate();

bool tofOk(int index);     // sensor initialised AND producing fresh data (false while stale/recovering)
int  tofRecoverCount();    // times a dropped-out ToF was brought back (telemetry)
bool rearBlocked();
void tofPrintRaw();        // 'u': toggle the live ToF line on/off
void tofPrintRawTick();    // call every loop() - no-op unless the live line is on

#endif /* __TOF_H */
