// smooth_servo.h - eased (cosine ease-in-out) servo moves.
//
// SmoothServo is NON-BLOCKING: start a move, call update() every loop, poll
// busy(). Use this in the collection FSM - the main loop (funnel sorting,
// ToFs, 8x8, round timer) keeps running while the crane swings.
//
// smoothServoWriteSlow() is the original BLOCKING helper, kept for bench
// sketches only. It stalls the whole program for the length of the move -
// don't call it from the FSM.
#pragma once
#include <Servo.h>

class SmoothServo
{
public:
    explicit SmoothServo(Servo &servo);

    void jumpTo(int angle);                        // immediate write, no easing
    void moveTo(int target, float degPerSec);      // eased move at an average speed
    void moveToIn(int target, unsigned long ms);   // eased move over a fixed time
    void update();                                 // call every loop
    bool busy() const { return moving; }
    int  angle() const { return current; }         // last angle written

private:
    void start(int target, unsigned long ms);

    Servo &servo;
    int current = 90;          // Servo library's default position before any write
    int from = 90, to = 90;
    unsigned long moveStart = 0, duration = 0, lastStep = 0;
    bool moving = false;
};

void smoothServoWriteSlow(Servo &servo, int targetAngle, float servo_deg_per_sec);
