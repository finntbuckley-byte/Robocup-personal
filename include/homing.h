#pragma once
void homingStartRound();
bool homingUpdate(bool pickupInProgress); // true: stopped/delivery owns motors; false: navigation runs
void homingStop();
void homingTelemetry();
void homingPrintStatus();

// Valid only after homingUpdate() delegates waypoint travel this tick.
bool homingNavigating();
float homingTargetHeading();
