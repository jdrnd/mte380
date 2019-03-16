#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>
#include "etl/queue.h"


#include "common.h"


#include "motor_control.h"

extern Task t_missionControl;

namespace MissionControl {
    void init();
    void run();
};
#endif