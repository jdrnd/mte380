#include <Arduino.h>

#include <Wire.h>
//#include <Plotter.h>

// Use this include because the author of the library was dumb
// and put C++ code in TaskScheduler.h so we get duplicated symbols
#include <TaskSchedulerDeclarations.h>

#include "actuators/drive_motors.h"
#include "actuators/servos.h"

#include "sensors/photosensor.h"
#include "sensors/imu.h"

#include "task/read_sensors.h"
#include "task/process_sensors.h"
#include "task/motor_control.h"
#include "task/mission_control.h"
#include "task/object_detection.h"
#include "sensors/gyro.h"
#include "common.h"//NEEDED FOR GYROS

Motors motors;
Photosensor candleSensor;
Rangefinders rangefinders;
Scheduler taskManager;

// Times in milliseconds
Task t_readSensors(50UL, TASK_FOREVER, &init_sensors, &taskManager, true);
Task t_motorControl(10UL, TASK_FOREVER, &MotorControl::init_motor_control, &taskManager, true);
<<<<<<< HEAD
Task t_missionControl(50UL, TASK_FOREVER, &MissionControl::init, &taskManager, true);
Task t_localize(50UL, TASK_FOREVER, &localize, &taskManager, true);
Task t_processSensors(50UL, TASK_FOREVER, &init_process_sensors, &taskManager, true);
=======
Task t_missionControl(100UL, TASK_FOREVER, &MissionControl::init, &taskManager, true);
Task t_localize(50UL, TASK_FOREVER, &localize, &taskManager, true);
Task t_processSensors(100UL, TASK_FOREVER, &init_process_sensors, &taskManager, true);
>>>>>>> working on wall following

// XBEE
// 3.3 V
// GND
// DOUT -> Serial3 Rx
// DIN -> Serial3 Tx
// Reset -> digital pin 6
#define XBEE_RESET_PIN 6

extern Servo armservo;

//int16_t leftValArray[5] = {0};//BB
//int8_t counter = 0;//BB

void setup() {
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
    // init_damper();
    // raise_damper();
    // delay(2000);
    // lower_damper();  
    // delay(2000);
    // fold_damper();
}


void loop() {
    taskManager.execute();
}
