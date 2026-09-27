#ifndef __ODOMETRY_H
#define __ODOMETRY_H
// ============================================================================
//  odometry.h  -  distance travelled from the drive-motor encoders, with
//  slip catching against the 8x8 (forward) / rear ToF (reverse).
//
//  Heading is NOT estimated here - that's the IMU's job. Homing will combine
//  IMU heading with odomDistanceMM() deltas to track x/y.
//  Pins, signs, scale and slip thresholds: config.h ODOMETRY.
// ============================================================================

void  odomInit();
void  odomUpdate();          // call every loop, after the ToF / 8x8 updates
void  odomReset();           // zero all distances (e.g. at round start)

long  encLeftCount();        // raw counts, sign-corrected (forward = up)
long  encRightCount();
float odomRawDistanceMM();   // encoders only, mean of both tracks, signed
float odomDistanceMM();      // slip-corrected, signed (forward +, reverse -)
bool  odomSlipping();        // last checked window looked like slip
bool  odomStalled();         // commanded to move, encoders not changing
unsigned odomSlipEvents();   // windows judged as slip since reset

#endif /* __ODOMETRY_H */
