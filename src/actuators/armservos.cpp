#include "armservos.h"

Servo armservo;

void init_arm_servo() {
    armservo.attach(SENSOR_ARM_PIN);
}

void lower_arm_servo() {
    armservo.write(ARM_ZERO_POSITION);
}

void raise_arm_servo() {
    armservo.write(ARM_RAISED_POSITION);
}