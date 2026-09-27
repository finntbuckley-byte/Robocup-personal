#ifndef __DRIVE_H
#define __DRIVE_H
// ============================================================================
//  drive.h  -  thin wrapper over motor.h (YOUR PARTNER'S CODE, unchanged)
//  so navigation.cpp can keep using the drive(left%, right%) style used
//  throughout this project, instead of motor.h's per-motor percent+number
//  calls directly. motor.h itself is untouched.
//
//  Reminder from motor.cpp: motor 1 = LEFT, motor 2 = RIGHT.
// ============================================================================

void drive(int leftPct, int rightPct);   // signed %, -100..100, differential
void driveForward();
void driveReverse();
void turnLeft();
void turnRight();
void stopMotors();        // soft stop: ramps down over DRIVE_DECEL_MS, call every loop
void driveHardStop();     // instant stop, no ramp - kill / rear guard only
bool isReversing();
int  lastDriveLeftPct();    // last commanded signed % (before pulse scaling)
int  lastDriveRightPct();

#endif /* __DRIVE_H */
