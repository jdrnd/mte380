#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Servo.h>

#define SENSOR_ARM_PIN 9

#define DAMPER_SERVO_1 8
#define DAMPER_SERVO_2 7

//#define DAMPER_ARM_PIN1 
//#define DAMPER_ARM_PIN2

#define ARM_ZERO_POSITION 85
#define ARM_RAISED_POSITION 180

void init_arm_servo();
void lower_arm_servo();
void raise_arm_servo();

void init_damper();
void lower_damper();
void raise_damper();

#endif SERVO_H
