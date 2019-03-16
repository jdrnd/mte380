#include "mission_control.h"

namespace MissionControl {
    // this task = t_missionControl
    void init() {
        DEBUG_PRINT("Init mission control");

        MotorControl::send_command(Command_t::DRIVE, 100);
        MotorControl::send_command(Command_t::TURN, 90);
        t_missionControl.setCallback(&run);
    }

    void run() {
        DEBUG_PRINT("Run mission control task");
    }
};
