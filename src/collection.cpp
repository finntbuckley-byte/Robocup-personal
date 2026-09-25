#include <Arduino.h>
#include <Servo.h>
#include "collection.h"
#include "Inductive_sensor.h"
#include "smooth_servo.h"
#include "config.h"   // CRANE_* angles and speeds

#define MAG1 26
#define MAG2 27

#define BIG_SERVO 28
#define SERVODELAY1 1200 //update after testing, motion from starting position to weight pickup
#define SERVODELAY2 900 //update after testing, motion from weight pickup to weight drop
#define SERVODELAY3 1500 //update after testing, motion from weight drop back to holding position
#define WEIGHTDROPDELAY 1000  //update after testing, time until weight is safely dropped

Servo bigServo;
static SmoothServo arm(bigServo); // non-blocking eased moves - see smooth_servo.h
static unsigned long collectionStateEntryTime;
static bool collectionStateEntry = false; //so we only run digitalwrite and servomove when we first enter the state
#define DROP_SETTLE_MS 1000 // pause before magnets off (was a blocking delay(1000)) - stops the weight being flung
// Crane angles + speeds live in config.h (CRANE_*). They're copied into
// `tuning` at boot so the servotest bench can change them live - see
// collection_tuning(). Each state waits for its move to FINISH before its
// dwell can end, so a slow speed just stretches the state, never cuts it.
static CraneTuning tuning = {
    CRANE_PICKUP_ANGLE, CRANE_PICKUP_DPS,
    CRANE_DROP_ANGLE,   CRANE_DROP_DPS,
    CRANE_REST_ANGLE,   CRANE_REST_DPS,
};

static bool magnetsReleased = false;
int servoPos = 0;

enum CollectionState
{
    COLLECTION_IDLE,
    COLLECTION_PICKUP,
    COLLECTION_MOVE_TO_DROP,
    COLLECTION_DROP,
    COLLECTION_FINISHED
};

static CollectionState state = COLLECTION_IDLE;


void collection_init(void)
{
    pinMode(MAG2, OUTPUT);
    pinMode(MAG1, OUTPUT);
    bigServo.attach(BIG_SERVO);
    arm.jumpTo(tuning.restAngle); // start parked (replaces the per-tick write(1) in collection_update)
}

// true for the whole pickup -> drop -> rest cycle; navigation holds still while set
bool collection_busy(void)
{
    return state != COLLECTION_IDLE;
}

// ---- bench hooks (servotest) - not used by navigation ----------------------
CraneTuning &collection_tuning(void) { return tuning; }

bool collection_move_to(int angle, float degPerSec)
{
    if (state != COLLECTION_IDLE) return false;   // never fight the FSM
    arm.moveTo(angle, degPerSec);
    return true;
}

bool collection_arm_busy(void)  { return arm.busy(); }
int  collection_arm_angle(void) { return arm.angle(); }

void collection_magnets(bool on)
{
    digitalWrite(MAG1, on ? HIGH : LOW);
    digitalWrite(MAG2, on ? HIGH : LOW);
}

void collection_start(void)
{
    if (state == COLLECTION_IDLE)
    {
        collectionStateEntryTime = millis();
        state = COLLECTION_PICKUP; //move into the pickup state which takes it from there
        Serial.println(F("[collection] START -> PICKUP"));
        collectionStateEntry = true; 
    }
    else 
    {
        Serial.println(F("[collection] start ignored, cycle already in progress"));
        return;
    } //don't want to start another cycle while picking up a different weight
}

void collection_update(void)
{
    // removed: bigServo.write(1) here ran every tick and overrode the 110 deg
    // pickup / 70 deg rest writes below
    arm.update(); // advance any eased move - every tick, never blocks
    switch (state)
    {
        case COLLECTION_IDLE:
            break;

        case COLLECTION_PICKUP:
            if (collectionStateEntry)
            {
                Serial.print(F("[collection] PICKUP: servo -> ")); Serial.print(tuning.pickupAngle);
                Serial.print(F(" deg @ ")); Serial.print(tuning.pickupDps, 0);
                Serial.print(F(" deg/s, magnets ON  (dwell "));
                Serial.print(SERVODELAY1);
                Serial.println(F("ms)"));
                arm.moveTo(tuning.pickupAngle, tuning.pickupDps); // eased
                digitalWrite(MAG1, HIGH);
                digitalWrite(MAG2, HIGH); //magnets are now on and will pick up weight when servo finishes moving
                collectionStateEntry = false;
            }

            if (millis() - collectionStateEntryTime >= SERVODELAY1 && !arm.busy())
            {
                Serial.println(F("[collection] PICKUP -> MOVE_TO_DROP"));
                state = COLLECTION_MOVE_TO_DROP;
                collectionStateEntryTime = millis();
                collectionStateEntry = true;
            }
            break;
        case COLLECTION_MOVE_TO_DROP:  
            if (collectionStateEntry)
            {
                Serial.print(F("[collection] MOVE_TO_DROP: easing ")); Serial.print(arm.angle());
                Serial.print(F(" -> ")); Serial.print(tuning.dropAngle);
                Serial.print(F(" deg @ ")); Serial.print(tuning.dropDps, 0);
                Serial.print(F(" deg/s  (dwell "));
                Serial.print(SERVODELAY2);
                Serial.println(F("ms)"));

                arm.moveTo(tuning.dropAngle, tuning.dropDps);
                collectionStateEntry = false;
            }

            // The eased swing (cosine ease-in-out, slow-fast-slow) runs in
            // SmoothServo::update() above, so the magnetically held weight isn't
            // flung. DROP (magnets off) waits for the swing to actually finish.
            if (millis() - collectionStateEntryTime >= SERVODELAY2 && !arm.busy())
            {
                Serial.println(F("[collection] MOVE_TO_DROP -> DROP"));
                state = COLLECTION_DROP;
                collectionStateEntryTime = millis();
                collectionStateEntry = true;
            }
            break;
        case COLLECTION_DROP:
            if (collectionStateEntry)
            {
                Serial.print(F("[collection] DROP: at drop point, magnets off in "));
                Serial.print(DROP_SETTLE_MS);
                Serial.println(F("ms"));
                magnetsReleased = false;
                collectionStateEntry = false;
            }

            // non-blocking version of the old delay(1000) before magnets off
            if (!magnetsReleased && millis() - collectionStateEntryTime >= DROP_SETTLE_MS)
            {
                digitalWrite(MAG1, LOW);
                digitalWrite(MAG2, LOW);
                magnetsReleased = true;
                Serial.println(F("[collection] DROP: magnets OFF - weight released"));
            }

            // same timing as before: WEIGHTDROPDELAY counts from DROP entry, so with
            // both at 1000 the crane heads to rest right after release. Raise it to
            // give the weight time to fall first.
            if (magnetsReleased && millis() - collectionStateEntryTime >= WEIGHTDROPDELAY)
            {
                Serial.println(F("[collection] DROP -> FINISHED"));
                state = COLLECTION_FINISHED;
                collectionStateEntryTime = millis();
                collectionStateEntry = true;
            }
            break;
        case COLLECTION_FINISHED:
            if (collectionStateEntry)
            {
                Serial.print(F("[collection] FINISHED: servo -> ")); Serial.print(tuning.restAngle);
                Serial.print(F(" deg (rest)  (dwell "));
                Serial.print(SERVODELAY3);
                Serial.println(F("ms)"));
                arm.moveTo(tuning.restAngle, tuning.restDps); // eased. placeholder, update with testing to location where crane is safe to rest
                collectionStateEntry = false;
            }
            if (millis() - collectionStateEntryTime >= SERVODELAY3 && !arm.busy())
            {
                Serial.println(F("[collection] FINISHED -> IDLE"));
                state = COLLECTION_IDLE;
                collectionStateEntryTime = millis();
                collectionStateEntry = true;
            }
            break;
    }
}



