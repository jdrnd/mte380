#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>
#include "etl/queue.h"

#include "common.h"

#include "motor_control.h"

#include "path_finder/path_finder.h"

#include "sensors/colorsensor.h"
#include "sensors/rangefinders.h"
#include "sensors/flame.h"

#include "actuators/servos.h"

#include "actuators/servos.h"

#define STARTING_X_POS 0
#define STARTING_Y_POS 0
#define STARTING_ORIENTATION 1 // along y-axis, towards center

extern ColorSensor colorsensor;
extern bool flameDetected;
extern Rangefinders rangefinders;

extern Task t_missionControl;

namespace MissionControl {

    extern int8_t orientation;
    extern Terrain map[6][6];
    extern PathFinder pathfinder;

    enum class State_t: uint8_t {
        NONE = 0,
        EXPLORE = 42,
        MOVE,
        CANDLE_HOMING = 25
    };

    extern State_t state;

    void init();
    void run();

    void do_candle_homing();
<<<<<<< HEAD
    void do_explore();
    void do_move_path();


    void get_front_square(uint8_t, uint8_t, int8_t, uint8_t&, uint8_t&);
=======
>>>>>>> initial candle homing
};
#endif