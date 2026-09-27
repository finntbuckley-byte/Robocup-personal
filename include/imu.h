#ifndef __IMU_H
#define __IMU_H
// ============================================================================
//  imu.h  -  BNO055 heading (IMUPLUS) + a heading-hold steering term.
//  Convention everywhere: + = turned RIGHT (clockwise seen from above).
//  Bus / address / sign / gains: config.h IMU.
// ============================================================================

bool  imuInit();              // false if the BNO055 isn't there - everything below then returns 0
void  imuUpdate();            // call every loop; reads every IMU_READ_MS
void  imuZero();              // heading = 0 here (e.g. at GO)
bool  imuOk();
float imuHeadingDeg();        // -180..180 since the last imuZero()
float imuRateDps();           // yaw rate, + = turning right
uint8_t imuGyroCal();         // BNO gyro calibration 0-3

// Steering (% differential, + = steer right) to bring the heading back to
// targetDeg. Use as drive(speed + s, speed - s). 0 if the IMU isn't ok.
// The integral only builds while this is being called every loop (a gap of
// > 100 ms, e.g. while steering on purpose, adds nothing). Reset it at the
// start of a run / round with headingHoldReset().
int   headingHoldSteer(float targetDeg);
void  headingHoldReset();
float headingHoldIntegral();  // the I term's current % (the learned drag bias)

#endif /* __IMU_H */
