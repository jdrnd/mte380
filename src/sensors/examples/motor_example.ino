#include <Arduino.h>
#include <Wire.h>

#include "actuators/drive_motors.h"

Motors motors;

// Wire single motor and set pins in motors.h
void setup() {
    Serial.begin(115200);

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
}

void loop() {
    motors.left->setSpeed(100);
    delay(2000);
    motors.left->setSpeed(-100);
    delay(2000);
    motors.left->stop();
    delay(2000);
}