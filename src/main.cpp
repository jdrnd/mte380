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
    delay(4000);
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
    while(true) {
        delay(100);
        Accel accel = imu->getAccel();
        Serial.print("Accel: ");
        Serial.print((float)accel.x,4);
        Serial.print(",");
        Serial.print((float)accel.y),4;
        Serial.print(",");
        Serial.println((float)accel.x,4);
    }
}
void loop() {
    doSensorDemo();
    //doMovementDemo();
    //while(true){}
}

