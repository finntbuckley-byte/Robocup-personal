#include <Arduino.h>
#include "motor.h"
#include "collection.h"

/*
IntervalTimer myTimer; //sets up a timer interrupt in case we want it

void myISR()
{
    //isr does stuff if we want it
}
void setup()
{
    Serial.begin(115200);
    motor_init();
    collection_init();
}

void loop()
{
    motorForward(10, 1);
    motorForward(10, 2);
    delay(5000); // hold long enough to visually confirm both are spinning
    //collection_update;
    /*
    if (weight is sensed and in correct position (ie inductive sensor etc.))
    {
        collection_start();
    }
    *//*
}*/

