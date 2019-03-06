#ifndef READ_SENSORS_H
#define READ_SENSORS_H

#include <TaskSchedulerDeclarations.h>

#include "actuators/motors.h"

#include "sensors/colorsensor.h"
#include "sensors/magnetic.h"
#include "sensors/imu.h"
#include "sensors/rangefinders.h"
#include "sensors/photosensor.h"

extern Rangefinders rangefinders;
// No need to include IMU here due to static nature
extern Photosensor candleSensor;
extern Task t_readSensors;

void init_sensors();
void read_sensors();

#endif