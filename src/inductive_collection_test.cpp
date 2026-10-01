// Isolated bench test of the real collection FSM. No navigation, gate or ToFs.
// LED on = metal; the existing interface presents this as active LOW on pin 20.
#ifdef INDUCTIVE_COLLECTION_TEST
#include <Arduino.h>
#include "config.h"
#include "collection.h"
#include "motor.h"

namespace {
bool stopped = false, armed = false, cycle = false;
bool timingClear = false, timingMetal = false, previousMetal = false, haveSample = false;
uint32_t clearAt = 0, metalAt = 0, cycleAt = 0, sampleAt = 0, reportAt = 0;
unsigned cycleNumber = 0;
int raw = 0, level = HIGH;
bool metal = false;

void resetTrigger() {
  armed = timingClear = timingMetal = false;
}

void status() {
  Serial.printf("INDUCTIVE t=%lu phase=%s cycle=%u pin=%d digital=%d adc=%d metal=%d arm=%d busy=%d cycle_ms=%lu\n",
    millis(), stopped ? "STOPPED" : cycle ? "COLLECTING" : armed ? "ARMED" : "WAIT_CLEAR",
    cycleNumber, PIN_INDUCTIVE, level, raw, metal, collection_arm_angle(), collection_busy(),
    cycle ? millis()-cycleAt : 0UL);
}

void stop(const char *reason) {
  collection_stop_motion();
  collection_magnets(false);
  motorStop(0);
  stopped = true; cycle = false; resetTrigger();
  Serial.printf("TEST_STOP: %s; crane frozen, magnet OFF. a resumes after clear.\n", reason);
}

void help() {
  Serial.println("INDUCTIVE_COLLECTION_V1 - automatic metal-triggered pickup");
  Serial.printf("Sensor LED ON = metal; pin=%d activeLow=%d; debounce=%lu ms; clear-to-rearm=%lu ms\n",
    PIN_INDUCTIVE, INDUCTIVE_ACTIVE_LOW, INDUCTIVE_CONFIRM_MS, METAL_REARM_CLEAR_MS);
  Serial.printf("Pickup=%d deg, seating=%lu ms. Normal pickup/drop/rest, full magnet power.\n",
    CRANE_PICKUP_ANGLE, CRANE_PICKUP_SEAT_MS);
  Serial.println("Keep crane travel clear. Remove target first; WAIT_CLEAR -> ARMED -> insert weight.");
  Serial.println("Drive held neutral; GO ignored; gate not commanded. x=stop/magnet OFF, a=resume, p=status, ?=help");
  Serial.println("Sensor changes during collection are logged but cannot cancel, retrigger or count a pickup.");
}
}

void setup() {
  motor_init(); motorStop(0);
  pinMode(PIN_INDUCTIVE, INPUT);
  analogReadResolution(12);
  collection_init(); // same rest pose and magnet-off initialization as nav
  Serial.begin(115200);
  const uint32_t began = millis();
  while (!Serial && millis()-began < 2000) {}
  help();
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'x': stop("operator requested"); break;
      case 'a':
        if (stopped) {stopped=false;resetTrigger();Serial.println("Resumed: waiting for clear sensor.");}
        break;
      case 'p': status(); break;
      case '?': help(); break;
      default: break;
    }
  }
  if (!stopped) collection_update();
  const uint32_t now = millis();
  if (now-sampleAt < 5) return;
  // A long sampling gap must not count as evidence of a stable input.
  if (now-sampleAt > PICKUP_VERIFY_MAX_SAMPLE_GAP_MS) timingClear=timingMetal=false;
  sampleAt=now;
  level=digitalRead(PIN_INDUCTIVE);
  raw=analogRead(PIN_INDUCTIVE);
  metal=INDUCTIVE_ACTIVE_LOW ? level==LOW : level==HIGH;
  if (!haveSample || metal!=previousMetal) {
    Serial.printf("SENSOR_EDGE t=%lu metal=%d digital=%d adc=%d cycle=%u cycle_ms=%lu\n",
      now,metal,level,raw,cycleNumber,cycle?now-cycleAt:0UL);
    previousMetal=metal;haveSample=true;
  }
  if (!stopped && cycle) {
    if (!collection_busy()) {
      cycle=false;resetTrigger();
      Serial.printf("CYCLE_COMPLETE number=%u elapsed_ms=%lu metal=%d; physical pickup success NOT inferred\n",
        cycleNumber, now-cycleAt, metal);
    } else if (now-cycleAt >= PICKUP_TIMEOUT_MS) {
      stop("collection timeout");
    }
  } else if (!stopped) {
    if (!armed) {
      if (metal) timingClear=false;
      else {
        if (!timingClear) {timingClear=true;clearAt=now;}
        if (now-clearAt>=METAL_REARM_CLEAR_MS) {
          armed=true;timingMetal=false;Serial.println("ARMED: present a weight to trigger collection.");
        }
      }
    } else if (!metal) timingMetal=false;
    else {
      if (!timingMetal) {timingMetal=true;metalAt=now;}
      if (now-metalAt>=INDUCTIVE_CONFIRM_MS) {
        resetTrigger();cycleAt=now;cycle=true;++cycleNumber;
        Serial.printf("TRIGGER t=%lu cycle=%u metal=%d adc=%d\n",now,cycleNumber,metal,raw);
        collection_start(false); // normal collection, no third-weight hold
      }
    }
  }
  if (now-reportAt>=100) {reportAt=now;status();}
}
#endif
