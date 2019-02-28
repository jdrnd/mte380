#include <Arduino.h>

#include <Wire.h>

#include "actuators/motors.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

Motors motors;

Photosensor candleSensor;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C
    pinMode(LED_BUILTIN, OUTPUT);

    imu->init();
    imu->run();

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
    motors.right->init(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_SENSOR_A_PIN, RIGHT_MOTOR_SENSOR_B_PIN, REVERSE);
}

void doMovementDemo() {
    delay(2000);
    motors.left->setSpeed(100);
    motors.right->setSpeed(100);
    delay(2000);
    motors.left->stop();
    motors.right->stop();
    delay(2000);
    motors.left->setSpeed(-100);
    motors.right->setSpeed(-100);
    delay(2000);
    motors.left->stop();
    motors.right->stop();
    delay(2000);

    motors.left->setSpeed(100);
    motors.right->setSpeed(-100);
    delay(2000);
    motors.left->stop();
    motors.right->stop();
    motors.left->setSpeed(-100);
    motors.right->setSpeed(100);
    delay(2000);
    motors.left->stop();
    motors.right->stop();
}

void doSensorDemo() {
    for (int i=0; i<200; i++) {
        Accel accel = imu->getAccel();
    }

    int average = 0;
    for (int i=0; i<100; i++) {
        average += candleSensor.read();
    }
    average = average / 100;

    while(true) {
        delay(200);
        Accel accel = imu->getAccel();
        //Serial.print("Accel: ");
        Serial.print((float)accel.x,4);
        Serial.print(",");
        Serial.print((float)accel.y),4;
        Serial.print(",");
        Serial.println((float)accel.z,4);

        if (abs(accel.x) > 0.15 || abs(accel.y) > 0.15 || abs(accel.z) > 0.15) {
            motors.left->setSpeed(-100);
            motors.right->setSpeed(-100);
            delay(200);
            motors.left->stop();
            motors.right->stop();
            delay(1000);
        }

        Serial.print("Photo: ");
        candleSensor.logLastReading();

        if (candleSensor.read()  > 750) {
            motors.left->setSpeed(100);
            motors.right->setSpeed(100);
            delay(200);
            motors.left->stop();
            motors.right->stop();
            delay(1000);
        }
    }
}
void loop() {
    doMovementDemo();
    doSensorDemo();
    while(true){}
}

