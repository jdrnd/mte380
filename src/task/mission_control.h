#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>
#include "etl/queue.h"


#include "common.h"


#include "motor_control.h"

#include "sensors/colorsensor.h"
#include "sensors/rangefinders.h"
#include "sensors/flame.h"

#include "actuators/servos.h"


extern ColorSensor colorsensor;
extern bool flameDetected;
extern Rangefinders rangefinders;

extern Task t_missionControl;

namespace MissionControl {

    enum class State_t: uint8_t {
        NONE = 0,
        CANDLE_HOMING = 25
    };

    extern State_t state;

    void init();
    void run();

    void do_candle_homing();
};
#endif