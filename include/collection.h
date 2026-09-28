#ifndef __COLLECTION_H
#define __COLLECTION_H

void collection_init(void);
void collection_update(void);
void magnet_collect(void);
// holdAtRest: skip the normal drop-into-storage and instead ease the arm to
// rest and hold the magnet there at MAGNET_HOLD_PCT indefinitely (does not
// turn off - not even at round end - only a power cycle drops it). For the
// 3rd target, carried on the arm instead of stored. See BENCH_TODO.md 2d.
void collection_start(bool holdAtRest = false);
bool collection_busy(void);

// Crane angles (deg) + speeds (deg/s). Start from config.h CRANE_*; the
// servotest bench changes them live through collection_tuning().
struct CraneTuning
{
    int   pickupAngle; float pickupDps;
    int   dropAngle;   float dropDps;
    int   restAngle;   float restDps;
};

// bench hooks (servotest) - navigation doesn't use these
CraneTuning &collection_tuning(void);
bool collection_move_to(int angle, float degPerSec);  // eased; refused mid-cycle
bool collection_arm_busy(void);
int  collection_arm_angle(void);
void collection_magnets(bool on);

#endif /*__COLLECTION_H*/