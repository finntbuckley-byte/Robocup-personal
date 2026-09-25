// smooth_servo.cpp - eased servo moves (non-blocking SmoothServo + the
// original blocking helper). Same cosine ease-in-out as the collection
// drop ramp: slow-fast-slow, so a magnetically held weight isn't flung.
#include "smooth_servo.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

static int clampAngle(int a) { return a < 0 ? 0 : (a > 180 ? 180 : a); }

SmoothServo::SmoothServo(Servo &s) : servo(s) {}

void SmoothServo::jumpTo(int angle)
{
    current = to = from = clampAngle(angle);
    moving = false;
    servo.write(current);
}

// The Servo library clamps to 0-180 anyway; clamping here keeps current/busy
// honest (a -5 target would otherwise never be "reached").
void SmoothServo::start(int target, unsigned long ms)
{
    from = current;
    to = clampAngle(target);
    duration = ms < SERVO_MIN_MOVE_MS ? SERVO_MIN_MOVE_MS : ms;
    moveStart = millis();
    lastStep = 0;
    moving = (to != from);
    if (!moving) servo.write(to);
}

void SmoothServo::moveTo(int target, float degPerSec)
{
    int distance = abs(clampAngle(target) - current);
    start(target, (unsigned long)((distance / degPerSec) * 1000.0f));
}

void SmoothServo::moveToIn(int target, unsigned long ms)
{
    start(target, ms);
}

void SmoothServo::update()
{
    if (!moving) return;
    unsigned long now = millis();
    if (now - lastStep < SERVO_STEP_INTERVAL_MS && lastStep != 0) return;
    lastStep = now;

    float t = (float)(now - moveStart) / (float)duration;
    if (t >= 1.0f)
    {
        current = to;
        moving = false;
    }
    else
    {
        float easedT = (1.0f - cos(t * PI)) / 2.0f;
        current = from + (int)((to - from) * easedT);
    }
    servo.write(current);
}

// Original blocking version (partner's), kept for bench sketches. Stalls the
// whole program for the move - never call it from the collection FSM.
void smoothServoWriteSlow(Servo &servo, int targetAngle, float servo_deg_per_sec)
{
    SmoothServo s(servo);
    s.jumpTo(servo.read());            // start from wherever it was last written
    s.moveTo(targetAngle, servo_deg_per_sec);
    while (s.busy())
    {
        s.update();
        delay(1);
    }
}
