#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

#include "actuators/drive_motors.h"

#include "common.h"

#define MOTOR_PROPORTIONAL_CONTSTANT 5

#include "sensors/imu.h"
#include "sensors/magnetic.h"

extern Task t_motorControl;
extern Motors motors;

extern Magnetics magnetics;

enum class Command: uint8_t {
    DRIVE,
    TURN
};

void init_motor_control();


void motor_control();

#endif