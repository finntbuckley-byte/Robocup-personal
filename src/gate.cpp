
#include <Arduino.h>
#include "gate.h"
#include <HerkulexServo.h>

#define GATE_SERVO 1

// Serial1 (D0/D1, CON65) is the drive motors' PPM pins, and CON67 (Serial7,
// pin 28) carries the crane servo - opening a UART on either kills those
// pulses. The Herkulex goes through the digital level-shift board to
// CON66 = SERIAL2 (RX2 D7, TX2 D8). Confirmed on the board 2026-09-25.
#define GATE_SERIAL Serial2

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