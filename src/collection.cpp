#include <Arduino.h>
#include <Servo.h>
#include "collection.h"
#include "Inductive_sensor.h"

#define MAG1 26
#define MAG2 27

#define BIG_SERVO 28
#define SERVODELAY1 1200 //update after testing, motion from starting position to weight pickup
#define SERVODELAY2 900 //update after testing, motion from weight pickup to weight drop
#define SERVODELAY3 1500 //update after testing, motion from weight drop back to holding position
#define WEIGHTDROPDELAY 1000  //update after testing, time until weight is safely dropped

Servo bigServo;
static unsigned long collectionStateEntryTime;
static bool collectionStateEntry = false; //so we only run digitalwrite and servomove when we first enter the state
#define MOVE_TO_DROP_START_ANGLE 116
#define MOVE_TO_DROP_END_ANGLE   -5 //previously 15
#define MOVE_TO_DROP_RAMP_MS     800  // must stay < SERVODELAY2 — see note below
#define AT_REST 70 //previously 90

static unsigned long moveToDropRampStart = 0;
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
    switch (state)
    {
        case COLLECTION_IDLE:
            break;

        case COLLECTION_PICKUP:
            if (collectionStateEntry)
            {
                Serial.print(F("[collection] PICKUP: servo -> 110 deg, magnets ON  (dwell "));
                Serial.print(SERVODELAY1);
                Serial.println(F("ms)"));
                bigServo.write(110);
                digitalWrite(MAG1, HIGH);
                digitalWrite(MAG2, HIGH); //magnets are now on and will pick up weight when servo finishes moving
                collectionStateEntry = false;
            }

            if (millis() - collectionStateEntryTime >= SERVODELAY1)
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
                Serial.print(F("[collection] MOVE_TO_DROP: easing 110 -> 15 deg  (dwell "));
                Serial.print(SERVODELAY2);
                Serial.println(F("ms)"));

                moveToDropRampStart = millis();
                collectionStateEntry = false;
            }

            // drive the eased position every tick, no delay() — runs alongside everything else in loop()
            // Non-blocking eased ramp: moves the servo from MOVE_TO_DROP_START_ANGLE to
            // MOVE_TO_DROP_END_ANGLE over MOVE_TO_DROP_RAMP_MS using a cosine ease-in-out
            // (slow-fast-slow) instead of a single write() jump. This kills the payload
            // swing that a hard start/stop caused, since the weight is only held by
            // magnetic shear force and slips under sharp acceleration/deceleration.
            // Runs one tick per collection_update() call — no delay() — so it doesn't
            // block the rest of the FSM or main loop.
            // IMPORTANT: MOVE_TO_DROP_RAMP_MS must stay LESS than SERVODELAY2, or the
            // state will advance to DROP (magnets off) before the servo/payload has
            // actually finished moving and settling.
            {
                unsigned long rampElapsed = millis() - moveToDropRampStart;
                if (rampElapsed <= MOVE_TO_DROP_RAMP_MS)
                {
                    float t = (float)rampElapsed / (float)MOVE_TO_DROP_RAMP_MS;
                    if (t > 1.0f) t = 1.0f;
                    float easedT = (1.0f - cos(t * PI)) / 2.0f;
                    int angle = MOVE_TO_DROP_START_ANGLE +
                                (int)((MOVE_TO_DROP_END_ANGLE - MOVE_TO_DROP_START_ANGLE) * easedT);
                    bigServo.write(angle);
                }
            }

            if (millis() - collectionStateEntryTime >= SERVODELAY2)
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
                Serial.print(F("[collection] DROP: magnets OFF  (dwell "));
                Serial.print(WEIGHTDROPDELAY);
                Serial.println(F("ms)"));
                Serial.print("waiting at end");
                delay(1000); // added this so that small pause in crane movement before weight drops before was flinging the weight
                digitalWrite(MAG1, LOW);
                digitalWrite(MAG2, LOW);
                collectionStateEntry = false;
            }

            if (millis() - collectionStateEntryTime >= WEIGHTDROPDELAY)
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
                Serial.print(F("[collection] FINISHED: servo -> 90 deg (rest)  (dwell "));
                Serial.print(SERVODELAY3);
                Serial.println(F("ms)"));
                bigServo.write(AT_REST); //placeholder, update with testing to location where crane is safe to rest
                collectionStateEntry = false;
            }
            if (millis() - collectionStateEntryTime >= SERVODELAY3)
            {
                Serial.println(F("[collection] FINISHED -> IDLE"));
                state = COLLECTION_IDLE;
                collectionStateEntryTime = millis();
                collectionStateEntry = true;
            }
            break;
    }
}



