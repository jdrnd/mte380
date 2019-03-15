#include "servos.h"

Servo armservo;

Servo damper1;
Servo damper2;

void init_arm_servo() {
    armservo.attach(SENSOR_ARM_PIN);
}

void lower_arm_servo() {
    armservo.attach(SENSOR_ARM_PIN);
    armservo.write(ARM_ZERO_POSITION);
    //pinMode(SENSOR_ARM_PIN, INPUT);
}

void raise_arm_servo() {
    armservo.attach(SENSOR_ARM_PIN);
    armservo.write(ARM_RAISED_POSITION);
    //pinMode(SENSOR_ARM_PIN, INPUT);
}

void init_damper() {
    damper1.attach(DAMPER_SERVO_1);
    damper2.attach(DAMPER_SERVO_2);
}

void lower_damper() {
    damper1.write(0);
    damper2.write(180);
}

void raise_damper() {
    damper1.write(90);
    damper2.write(90);
}