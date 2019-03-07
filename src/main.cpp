#include <Arduino.h>

#include <Wire.h>
#include <Plotter.h>

// Use this include because the author of the library was dumb
// and put code in TaskScheduler.h so we get duplicated symbols
#include <TaskSchedulerDeclarations.h>

#include "actuators/motors.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

<<<<<<< HEAD
#include "task/read_sensors.h"
#include "task/process_sensors.h"
#include "task/motor_control.h"
=======
#include "path_finder/path_finder.h"

Motors motors;
>>>>>>> Added initial path finding code with turning penalty.

#include "common.h"

Motors motors;
Photosensor candleSensor;
Rangefinders rangefinders;


Scheduler taskManager;

// Times in milliseconds
Task t_readSensors(100UL, TASK_FOREVER, &init_sensors, &taskManager, true);
Task t_processSensors(100UL, TASK_FOREVER, &init_process_sensors, &taskManager, true);
Task t_motorControl(1000UL, TASK_FOREVER, &init_motor_control, &taskManager, true);

// XBEE
// 3.3 V
// GND
// DOUT -> Serial3 Rx
// DIN -> Serial3 Tx
// Reset -> digital pin 6
#define XBEE_RESET_PIN 6

void setup() {
    Serial.begin(115200);
    Serial3.begin(116200);
    pinMode(XBEE_RESET_PIN, OUTPUT);
    digitalWrite(XBEE_RESET_PIN, LOW);
    delay(1);
    digitalWrite(XBEE_RESET_PIN, HIGH);
}

<<<<<<< HEAD
void loop() {
    taskManager.execute();
=======
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

void doPathFinderDemo() {
    Serial.println("\n   PATH FINDER DEMO");
    
    PathFinder pathFinder;
    pathFinder.init();
    pathFinder.printMapTerrain();
    pathFinder.planPath(2,2);
    pathFinder.printMapParents();
    pathFinder.printMapFCosts();
    //printMapFCosts
}

void loop() {
    doPathFinderDemo();
    while(true){}

    doMovementDemo();
    doSensorDemo();
    while(true){}
>>>>>>> Added initial path finding code with turning penalty.
}

