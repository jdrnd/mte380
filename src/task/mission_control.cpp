#include "mission_control.h"

namespace MissionControl {
    // this task = t_missionControl

    void init() {
        DEBUG_PRINT("Init mission control");
        /*
        MotorControl::send_command(Command_t::DRIVE, 100);
        MotorControl::send_command(Command_t::TURN, -90);
        */
        t_missionControl.setCallback(&run);
    }

    void run() {
        DEBUG_PRINT("Run mission control task");
        /*
        if (colorsensor.curr_terrain == Terrain::WATER && MotorControl::current_command.type == Command_t::DRIVE) {
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, 90);
        }
        */
    }
};
