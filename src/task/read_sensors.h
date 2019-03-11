#ifndef READ_SENSORS_H
#define READ_SENSORS_H

#include <TaskSchedulerDeclarations.h>

#include "actuators/drive_motors.h"

#include "sensors/colorsensor.h"
#include "sensors/magnetic.h"
#include "sensors/imu.h"
#include "sensors/rangefinders.h"
#include "sensors/photosensor.h"

#include "actuators/drive_motors.h"

extern Rangefinders rangefinders;
// No need to include IMU here due to static nature
extern Photosensor candleSensor;
extern Motors motors;

extern Task t_readSensors;

void init_sensors();
void read_sensors();

#endif