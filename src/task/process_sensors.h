#ifndef PROCESS_SENSORS_H
#define PROCESS_SENSORS_H

#include <TaskSchedulerDeclarations.h>
#include "common.h"

#include "actuators/drive_motors.h"

#include "sensors/colorsensor.h"
#include "sensors/magnetic.h"
#include "sensors/imu.h"
#include "sensors/rangefinders.h"
#include "sensors/photosensor.h"
#include "object_detection.h"

extern Task t_processSensors;

extern Motors motors;
extern Rangefinders rangefinders;
extern Photosensor candleSensor;
extern bool flameDetected;
//localization end

//extern IMU* imu;

extern int16_t LEFTLIDARVAL;

void init_process_sensors();
void process_sensors();
void print_object_data();

#endif