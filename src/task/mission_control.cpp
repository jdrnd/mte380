#include "mission_control.h"

namespace MissionControl {
    // this task = t_missionControl

    State_t state = State_t::NONE;
    static uint64_t count = 0;

    Terrain map[6][6];
    uint8_t x_pos;
    uint8_t y_pos;
    int8_t orientation = 0;

    void init() {
        DEBUG_PRINT("Init mission control");

        map[STARTING_X_POS][STARTING_Y_POS] = Terrain::WOOD;

        MotorControl::send_command(Command_t::DRIVE, 30);
        MotorControl::send_command(Command_t::TURN, 90);
        MotorControl::send_command(Command_t::DRIVE, 30);
        MotorControl::send_command(Command_t::TURN, -90);
        MotorControl::send_command(Command_t::TURN, 60);
        // MotorControl::send_command(Command_t::TURN, -90);
        // MotorControl::send_command(Command_t::TURN, -90);
        // MotorControl::send_command(Command_t::TURN, 90);
        // MotorControl::send_command(Command_t::TURN, 90);
        // MotorControl::send_command(Command_t::TURN, 90);
        // MotorControl::send_command(Command_t::TURN, 90);
        t_missionControl.setCallback(&run);
    }

    void run() {
        DEBUG_PRINT("Run mission control task");


        switch(state) {
            case State_t::CANDLE_HOMING:
                do_candle_homing();
                break;
            case State_t::EXPLORE:
                do_explore();
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
    
    void do_explore() {

    }


    void get_front_square(uint8_t curr_x, uint8_t curr_y, int8_t orientation, uint8_t& x, uint8_t& y) {
        // Assume we are never facing a wall

        // Transform to a positive orientation between 0 and 3
        orientation %= 4;


        switch (orientation) {
            case 0:
                x = curr_x;
                y = curr_y + 1;
                break;
            case 1:
                x = curr_x + 1;
                y = curr_y;
                break;
            case 2:
                x = curr_x;
                y = curr_y -1;
                break;
            case 3:
                x = curr_x -1;
                y = curr_y;
                break;
            default:
                break;
        }
    }
};
