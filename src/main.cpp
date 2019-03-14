#include <Arduino.h>

#include <Wire.h>
#include <Plotter.h>

// Use this include because the author of the library was dumb
// and put code in TaskScheduler.h so we get duplicated symbols
#include <TaskSchedulerDeclarations.h>

#include "actuators/drive_motors.h"
#include "actuators/armservos.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

#include "task/read_sensors.h"
#include "task/process_sensors.h"
#include "task/motor_control.h"

#include "common.h"

Motors motors;
Photosensor candleSensor;
Rangefinders rangefinders;
ColorSensor colorsensor;

Scheduler taskManager;

// Times in milliseconds
Task t_readSensors(100UL, TASK_FOREVER, &init_sensors, &taskManager, true);
Task t_processSensors(100UL, TASK_FOREVER, &init_process_sensors, &taskManager, true);
Task t_motorControl(10UL, TASK_FOREVER, &init_motor_control, &taskManager, true);

// XBEE
// 3.3 V
// GND
// DOUT -> Serial3 Rx
// DIN -> Serial3 Tx
// Reset -> digital pin 6
#define XBEE_RESET_PIN 6

extern Servo armservo;

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    // pinMode(XBEE_RESET_PIN, OUTPUT);
    // digitalWrite(XBEE_RESET_PIN, LOW);
    // delay(1);
    // digitalWrite(XBEE_RESET_PIN, HIGH);

    init_sensors();

    armservo.attach(SENSOR_ARM_PIN);
    init_arm_servo();
    lower_arm_servo();   
}

void loop() {
    //taskManager.execute();
    
    motors.setSpeed(50);
    colorsensor.read_terrain(true);
    while(colorsensor.curr_terrain == Terrain::WOOD)
    {
        colorsensor.read_terrain(rangefinders.shortrange.last_reading);
        delay(100);
    }
    motors.stop();
}