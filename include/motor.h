#ifndef __MOTOR_H
#define __MOTOR_H

#include <Servo.h>

void motor_init(void);
void motorForward(int percent, int motor);
void motorBackward(int percent, int motor);
void motorStop(int motor);


#endif /* __MOTOR_H*/