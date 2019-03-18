#include "mission_control.h"

namespace MissionControl {
    // this task = t_missionControl

    State_t state = State_t::CANDLE_HOMING;
    static uint64_t count = 0;

    void init() {
        DEBUG_PRINT("Init mission control");

        MotorControl::send_command(Command_t::DRIVE, 75);
        MotorControl::send_command(Command_t::TURN, -90);
        MotorControl::send_command(Command_t::DRIVE, 50);
        t_missionControl.setCallback(&run);
    }

    void run() {
        DEBUG_PRINT("Run mission control task");


        switch(state) {
            case State_t::CANDLE_HOMING:
                do_candle_homing();
                break;
            default:
                break;
        };
        count++;

        if (count == 3) {
            fold_damper();
            deinit_damper();
        }
        // if (colorsensor.curr_terrain == Terrain::WATER && MotorControl::current_command.type == Command_t::DRIVE) {
        //     MotorControl::stopMotors();
        //     MotorControl::command_queue.clear();
        //     MotorControl::send_command(Command_t::TURN, 90);
        // }
    }

    void do_candle_homing() {
        DEBUG_PRINT("Candle homing")
        static bool turning = false;
        static bool candleFound = false;

        static bool positioning_done = false;

        if (MotorControl::command_queue.size() > 0) return;

        if (!candleFound && MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {
            DEBUG_PRINT("scanning for candle");
            resetFlameDetection();
            MotorControl::send_command(Command_t::TURN, -360);
            positioning_done = true;
        }

        if(!candleFound && flameDetected) {
            DEBUG_PRINT("candle detected");
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, 10);
            candleFound = true;
            resetFlameDetection();
        }

        if (candleFound && rangefinders.front.last_reading > 200) {
            DEBUG_PRINT("moving towards candle");
            MotorControl::send_command(Command_t::DRIVE, 10);
        }

        if (candleFound && rangefinders.front.last_reading < 200 && flameDetected) {
            DEBUG_PRINT("exinguishing candle");
            MotorControl::stopMotors();
            //init_damper();
            //lower_damper();

            state = State_t::NONE;
            count = 0;
        }
    }
};
