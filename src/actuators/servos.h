#include <Servo.h>

#define SENSOR_ARM_PIN 9

#define ARM_ZERO_POSITION 115
#define ARM_RAISED_POSITION 180

void init_arm_servo();
void lower_arm_servo();
void raise_arm_servo();
