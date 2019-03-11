#ifndef PROCESS_SENSORS_H
#define PROCESS_SENSORS_H

#include <TaskSchedulerDeclarations.h>
#include <Plotter.h>

#include "common.h"

#include "actuators/drive_motors.h"

#include "sensors/colorsensor.h"
#include "sensors/magnetic.h"
#include "sensors/imu.h"
#include "sensors/rangefinders.h"
#include "sensors/photosensor.h"

extern Task t_processSensors;

extern Motors motors;
extern Rangefinders rangefinders;
extern Photosensor candleSensor;
//extern IMU* imu;

void init_process_sensors();
void process_sensors();

#endif