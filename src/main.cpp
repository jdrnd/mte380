#include <Arduino.h>

#include <Wire.h>
#include <Plotter.h>

// Use this include because the author of the library was dumb
// and put code in TaskScheduler.h so we get duplicated symbols
#include <TaskSchedulerDeclarations.h>

#include "actuators/motors.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

#include "task/read_sensors.h"
#include "task/process_sensors.h"
#include "task/motor_control.h"

#include "common.h"

Motors motors;
Photosensor candleSensor;
Rangefinders rangefinders;

bool objects[36];
int16_t confidence[36];

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

void loop() {
    taskManager.execute();
}

