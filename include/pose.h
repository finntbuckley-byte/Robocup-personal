#ifndef __POSE_H
#define __POSE_H
// ============================================================================
//  pose.h  -  where the robot is relative to where the round started.
//
//  Dead reckoning: each update, the slip-corrected encoder distance since the
//  last update (odometry.h) is split along the IMU heading (imu.h).
//  Frame (reset at GO): origin = start position = HOME,
//    +x = the direction the robot faced at GO, +y = to its RIGHT,
//    heading 0 = +x, + = turned right (same convention as imu.h).
//  Home is always (0,0), so homing doesn't need to know which corner or
//  base colour the round started in.
//
//  2026-09-28: LOGGING ONLY (px/py telemetry) - validate the drift in the
//  arena before homing or the rejected-dummy memory relies on it.
//  (Idea from the elise-n-2431/RoboCup_Codebase pose.cpp.)
// ============================================================================

void  poseReset();            // at GO, after odomReset() + imuZero()
void  poseUpdate();           // every loop, after odomUpdate() / imuUpdate()

float poseXmm();              // forward of the start position
float poseYmm();              // right of the start position
float poseDistHomeMM();       // straight-line distance back to the start
float poseBearingHomeDeg();   // heading (imu.h convention) that points at home

#endif /* __POSE_H */
