#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

#include "actuators/motors.h"

#include "common.h"

extern Task t_motorControl;
extern Motors motors;

void init_motor_control();
void motor_control();

#endif