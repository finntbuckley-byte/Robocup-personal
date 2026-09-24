#include <Arduino.h>
#include <Wire.h>
#include "motor.h"          // partner's code, unchanged
#include "collection.h"     // partner's code, + collection_busy() addition
#include "drive.h"
#include "tof.h"
#include "ir_sensors.h"
#include "funnel_sensor.h"
#include "weight_detect.h"
#include "gate.h"
//#include "navigation.h"
#include "config.h"
#include "Inductive_sensor.h"

unsigned long startTime;
bool collectionStarted = false;

// ============================================================================
//  main.cpp  -  full RoboCup robot: navigation + weight-seek + real pickup
//  + funnel sorting classification.
//
//  MODULE OWNERSHIP:
//    motor.cpp / collection.cpp   - your partner's code, unchanged (except
//                                    the header/collection_busy() addition
//                                    noted in those files)
//    everything else              - this build: navigation, sensing, sorting
//
//  LOOP STRUCTURE (nothing here blocks - every *_update() rate-limits
//  itself internally):
//    1. freshen every sensor (ToF, side IR)
//    2. fuse the low-vs-top ToF differential into a stable weight candidate
//    3. run collection_update() - always, so the crane FSM can progress
//       regardless of what navigation is doing (navigation just starts it
//       and waits via collection_busy())
//    4. run funnelSortUpdate() - always and independently, see
//       funnel_sensor.h for why
//    5. navigationUpdate() - decide + act
//    6. telemetry
//
//  BEFORE YOU FLASH: check every PLACEHOLDER pin in config.h against your
//  actual wiring - corner/rear ToF XSHUT, all 3 IR analog pins, and the
//  inductive digital pin were not specified and are best-guess defaults.
// ============================================================================

//static unsigned long lastTelemetry = 0;

/*static void printTelemetry()
{
  if (millis() - lastTelemetry < TELEMETRY_MS) return;
  lastTelemetry = millis();

  Serial.print("ToF low:"); Serial.print(tofBL); Serial.print("/"); Serial.print(tofBR);
  Serial.print(" top:");    Serial.print(tofFL); Serial.print("/"); Serial.print(tofFR);
  Serial.print(" corner:"); Serial.print(cornerL); Serial.print("/"); Serial.print(cornerR);
  Serial.print(" rear:");   Serial.print(tofRear);
  Serial.print(" | IR side:"); Serial.print(irSideLMM); Serial.print("/"); Serial.print(irSideRMM);
  Serial.print(" funnel:");    Serial.print(weightInFunnel() ? "PRESENT" : "-");
  Serial.print(" | W:");
  if (weightFound)
  {
    Serial.print(weightDistMM); Serial.print("mm ");
    Serial.print(weightSide == 0 ? "C" : (weightSide < 0 ? "L" : "R"));
  }
  else Serial.print(targetSuppressed() ? "supp" : "-");
  Serial.print(" | picks:"); Serial.print(pickupAttempts);
  Serial.print(" real:");    Serial.print(realWeightCount);
  Serial.print(" dummy:");   Serial.print(dummyCount);
  Serial.print(" | ");       Serial.println(modeName());
}*/

void setup()
{
  Serial.begin(115200);
  startTime = millis();

  motor_init();          // partner's code
  collection_init();
  magnet_on_init();     // partner's code
  gate_init();
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  //tofInit();
  irSensorsInit();
  funnelSensorInit();

  Serial.println("--- RoboCup full build ready (7 ToF + 3 IR + inductive) ---");

  //navigationInit();
  delay(500);
}

void loop()
{
  // 1. freshen every sensor (each rate-limits itself; the loop never blocks)
  /*tofUpdate();
  irSensorsUpdate();
  // 2. fuse the low-vs-top ToF differential into a stable candidate
  weightDetectUpdate();*/

  // 3. crane/magnet FSM - always ticks, navigation just starts/watches it
    //collection_update();  // called every iteration, unconditionally, no delay
    //gate_move_test();
    //gate_update();
    collection_update();

    // trigger one pickup cycle on command, so you can watch each step happen
    /*if (Serial.available())
    {
        char c = Serial.read();
        if (c == 's')
        {
            collection_start();
        }
    }*/
   if (!collectionStarted && millis() - startTime >= 10000)
    {
        collection_start();
        Serial.print ("collection should have started");
        collectionStarted = true;
    }

  // 4. funnel sorting - independent of navigation/collection, see
  //    funnel_sensor.h for why
  /*funnelSortUpdate();

  // 5. decide + act
  navigationUpdate();

  // 6. tell the humans
  printTelemetry();
  //should_magnet_turn_on();*/

}
