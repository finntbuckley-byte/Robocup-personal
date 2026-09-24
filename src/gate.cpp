
#include <Arduino.h>
#include "gate.h"
#include <HerkulexServo.h>

#define GATE_SERVO 1

// Serial1 (D0/D1, CON65) is the drive motors' PPM pins - opening a UART there
// kills the motor pulses. The Herkulex goes through the digital level-shift
// board to CON67 = SERIAL7 (RX7 pin 28, TX7 pin 29).
// TODO(verify): collection.cpp drives the crane servo on pin 28 = RX7 - the
// same pin. Serial7.begin() takes pin 28 over as UART RX, so both can't work
// as written. Find where the crane servo's signal wire actually lands.
#define GATE_SERIAL Serial7

HerkulexServoBus herkulexBus(GATE_SERIAL);
HerkulexServo gateServo(herkulexBus, GATE_SERVO);

void gate_init(void)
{
    GATE_SERIAL.begin(115200);

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
    gateServo.setPosition(895, 70, HerkulexLed::Blue);

    // move 90 degrees clockwise
    gateServo.setPosition(613, 70, HerkulexLed::Green);
    delay(1000);

    // back to start pos
    gateServo.setPosition(895, 70, HerkulexLed::Blue);

}