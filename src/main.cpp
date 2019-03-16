#include <Arduino.h>

#include <Wire.h>
#include <Plotter.h>

// Use this include because the author of the library was dumb
// and put code in TaskScheduler.h so we get duplicated symbols
#include <TaskSchedulerDeclarations.h>

#include "actuators/drive_motors.h"
#include "actuators/servos.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

#include "task/read_sensors.h"
#include "task/process_sensors.h"
#include "task/motor_control.h"

#include "sensors/gyro.h"

#include "common.h"//NEEDED FOR GYROS

Motors motors;
Photosensor candleSensor;
Rangefinders rangefinders;

bool objects[36];
int16_t confidence[36];

ColorSensor colorsensor;
Magnetics magnetics;
Gyro gyro;

Scheduler taskManager;


// Times in milliseconds
Task t_readSensors(50UL, TASK_FOREVER, &init_sensors, &taskManager, true);
Task t_processSensors(50UL, TASK_FOREVER, &init_process_sensors, &taskManager, true);
Task t_motorControl(20UL, TASK_FOREVER, &init_motor_control, &taskManager, true);

// XBEE
// 3.3 V
// GND
// DOUT -> Serial3 Rx
// DIN -> Serial3 Tx
// Reset -> digital pin 6
 #define XBEE_RESET_PIN 6

extern Servo armservo;

void setup() {
    //init_damper();

    Serial.begin(115200);
    Serial3.begin(115200);
    pinMode(XBEE_RESET_PIN, OUTPUT);
    digitalWrite(XBEE_RESET_PIN, LOW);
    delay(1);
    digitalWrite(XBEE_RESET_PIN, HIGH);

    //init_arm_servo();
    //lower_arm_servo();
    //delay(2000);
    //raise_arm_servo();  
    delay(2000);
}


void loop() {
    taskManager.execute();
}