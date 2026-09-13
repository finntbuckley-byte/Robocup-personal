
#include <Arduino.h>
#include "gate.h"
#include <HerkulexServo.h>

#define GATE_SERVO 1

HerkulexServoBus herkulexBus(Serial1);
HerkulexServo gateServo(herkulexBus, GATE_SERVO);

void gate_init(void)
{
    Serial1.begin(115200);

    delay(100);

    gateServo.reboot();
    delay(100);

    gateServo.setTorqueOn();
    gateServo.enablePositionControlMode();
}

void gate_update(void)
{
    herkulexBus.update();
}

void gate_move_test(void)
{
    gateServo.setTorqueOn();
    gateServo.enablePositionControlMode();

    delay(5000);

    // start pos — 45 degrees more clockwise
    gateServo.setPosition(485, 70, HerkulexLed::Blue);

    // move 90 degrees clockwise
    gateServo.setPosition(213, 70, HerkulexLed::Green);
    delay(1000);

    // back to start pos
    gateServo.setPosition(485, 70, HerkulexLed::Blue);

}