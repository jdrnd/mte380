<<<<<<< HEAD
#ifdef teensy

#include <Arduino.h>

#include <Wire.h>

#include "actuators/motors.h"

Motors motors;

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

=======
#include "sensors/magnetic.h"


void setup() {
    Serial.begin(115200);
}

void loop() {
    delay(1000);
    if (Magnetics::detectMagnet()) {
        Serial.println("Magnet!");
    } else {
        Serial.println("No Magnet!");
    }
}
>>>>>>> Interfaced hall effect sensor
