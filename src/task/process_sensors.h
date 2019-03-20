#ifndef PROCESS_SENSORS_H
#define PROCESS_SENSORS_H

#include <TaskSchedulerDeclarations.h>
//#include <Plotter.h>

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
//extern ColorSensor colorsensor;
extern Magnetics magnetics;
extern bool flameDetected;

//localization variables
extern bool objects[36];
extern int16_t confidence[36];
extern uint16_t X;
extern uint16_t Y;
extern int16_t der_l;
extern int16_t der_r;
extern bool obj_l;
extern bool obj_r;
//localization end

//extern IMU* imu;

extern int16_t LEFTLIDARVAL;

void init_process_sensors();
void process_sensors();
void print_object_data();

#endif