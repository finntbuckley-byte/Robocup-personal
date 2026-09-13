#include "motor.h"
#include <Arduino.h>

// PPM pulse widths for DFR0513 (see datasheet)
#define FULL_BACKWARD  1000
#define MIN_BACKWARD   1188  //62% of the full pulse was min you would want to run the motor at
#define NEUTRAL       1500   // center of 1470-1560 stop deadband
#define MIN_FORWARD  1842
#define FULL_FORWARD 2000

#define MOTOR_1_PIN 0   // this is the left motor
#define MOTOR_2_PIN 1   // this is the right motor

Servo motor1;
Servo motor2;

void motor_init(void)
{
    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);

    // send neutral immediately so the driver doesn't see a random
    // pulse at power-up before your first real command
    motor1.writeMicroseconds(NEUTRAL);
    motor2.writeMicroseconds(NEUTRAL);
}

void motorForward(int percent, int motor)
{
    percent = constrain(percent, 0, 100);
    int time_forward = MIN_FORWARD + (percent * (FULL_FORWARD - MIN_FORWARD)) / 100;
    switch (motor) {
        case (1): motor1.writeMicroseconds(time_forward);
        break;
        case (2): motor2.writeMicroseconds(time_forward);
        break;
        default:
            Serial.print("motorForward: invalid motor number ");
            Serial.println(motor);
            break;
    }

}

void motorBackward(int percent, int motor)
{
    percent = constrain(percent, 0, 100);
    int time_backward = MIN_BACKWARD - (percent * (MIN_BACKWARD - FULL_BACKWARD)) / 100;
    switch (motor) {
        case (1): motor1.writeMicroseconds(time_backward);
        break;
        case (2): motor2.writeMicroseconds(time_backward);
        break;
        default: 
            Serial.print("motorBackward: invalid motor number ");
            Serial.println(motor);
            break;
    }


}

void motorStop(int motor)
{
    switch (motor) {
        case 1: motor1.writeMicroseconds(NEUTRAL);
        break;
        case 2: motor2.writeMicroseconds(NEUTRAL);
        break;
        default: 
            motor1.writeMicroseconds(NEUTRAL);
            motor2.writeMicroseconds(NEUTRAL);
            break;
    }
}

