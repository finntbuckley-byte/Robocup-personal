#ifndef __COLLECTION_H
#define __COLLECTION_H

void collection_init(void);
void collection_update(void);
void magnet_collect(void);
void collection_start(void);
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