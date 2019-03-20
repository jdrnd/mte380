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
#include "sensors/magnetic.h"

#include "actuators/servos.h"

#include "actuators/servos.h"

#define STARTING_X_POS 0
#define STARTING_Y_POS 0
#define STARTING_ORIENTATION 1 // along y-axis, towards center

extern ColorSensor colorsensor;
extern bool flameDetected;
extern Rangefinders rangefinders;
extern Magnetics magnetics;

extern Task t_missionControl;

#define STARTING_X_POS 5
#define STARTING_Y_POS 3
#define STARTING_ORIENTATION 2 // along y-axis, towards center

#define NUM_SCAN_POSITIONS 4
#define NUM_SAND_POSITIONS 3

struct Position {
    uint8_t x;
    uint8_t y;
    bool visited;
};

extern const Position sandpits[NUM_SAND_POSITIONS];
extern const Position scan_positions[NUM_SCAN_POSITIONS];

namespace MissionControl {
    extern bool magnet_found;
    extern int8_t orientation;
    extern Terrain map[6][6];
    extern PathFinder pathfinder;

    enum class State_t: uint8_t {
        NONE = 0,
        EXPLORE = 42,
        MOVE,
        FIND_MAGNET,
        CANDLE_SEARCH,
        CANDLE_HOMING = 25
    };

    extern State_t state;

    void init();
    void run();

    void do_candle_homing();
    void do_move_path();
    void do_find_magnet();
    void do_candle_search();


    void update_position(int16_t);
    void get_front_square(uint8_t, uint8_t, int8_t, uint8_t&, uint8_t&);

    void do_explore();

    void send_next_planned_move();
};
#endif