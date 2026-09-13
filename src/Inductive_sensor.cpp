#include "collection.h"
#include "Inductive_sensor.h"
#include <Arduino.h>
#include <Servo.h>


const int InducProxPin1 = 20;
#define INDUCPIN 20
#define MAG1 26
#define MAG2 27

#define METALLIC 0
#define NON_METAL 1

Servo kgServo;

int InducProxState = 0;
bool was_metallic = false;

void magnet_on_init(void)
{
    pinMode(INDUCPIN, INPUT);
    Serial1.begin(115200);
}

void should_magnet_turn_on(void)
{
  // put your main code here, to run repeatedly:
  InducProxState = digitalRead(INDUCPIN);
  Serial.println("new next reading");
  Serial.println(InducProxState);
  delay(500);

  

  if (InducProxState == METALLIC)
  {
    
      // Only run when we JUST became metallic
    if (!was_metallic)
    {
        was_metallic = true;

        collection_start();
    } 
    else
    {
        // Reset so it can trigger again next time metal is detected
        was_metallic = false;
    }
  }

  if (InducProxState == NON_METAL) 
  {
    //needs to trigger an escape plan
  }
}
