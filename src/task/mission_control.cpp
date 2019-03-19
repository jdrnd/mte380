#include "mission_control.h"

namespace MissionControl {
    // this task = t_missionControl

    State_t state = State_t::CANDLE_HOMING;
    static uint64_t count = 0;

    Terrain map[6][6];

    uint8_t x_pos = STARTING_X_POS;
    uint8_t y_pos = STARTING_Y_POS;
    int8_t orientation = STARTING_ORIENTATION;

    PathFinder pathfinder;

    PathFinder pathfinder;

    void init() {
        DEBUG_PRINT("Init mission control");
        pathfinder.init();

        //pathfinder.setBotPosition(STARTING_X_POS, STARTING_Y_POS, 1);
        pathfinder.setBotPosition(0, 3, 1);
        pathfinder.printMapTerrain();
        pathfinder.setTargetPosition(5,5);
        
        if (pathfinder.planPath()) {
            pathfinder.printMapParents();
            Serial.println("Steps: " + String(pathfinder.path.size()));
            String s = "";
            for(int8_t i = pathfinder.path.size() - 1; i >= 0; i--)
                s = s + String(pathfinder.plan[i]) + ",";
            DEBUG_PRINT(s);
        }

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
            case State_t::MOVE:
                do_move_path();
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
        static bool turningLeft = false;
        static bool turningRight = false;
        static bool approaching;
        static bool turning = false;
        static bool candleFound = false;

        static bool positioning_done = false;

        //if (MotorControl::command_queue.size() > 0) return;

        /*
        if (!candleFound && MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {
            DEBUG_PRINT("scanning for candle");
            resetFlameDetection();
            //MotorControl::send_command(Command_t::TURN, -360);
            //MotorControl::send_command(Command_t::TURN, -70);
            //MotorControl::send_command(Command_t::TURN, +70);
            positioning_done = true;
            approaching = false;
        }*/

        if(!candleFound && MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {
            if (turningRight) turningRight = false;
            if (turningLeft) turningLeft = false;
            if (approaching) approaching = false;
        }

        PLOTTER_SERIAL.println("approaching: " + String(approaching) + 
            " L: " + String(flameDetectedLeft) + " R: " + String(flameDetectedRight));
        if(!candleFound && flameDetectedLeft && flameDetectedRight && !approaching) {
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::DRIVE, 10);
            turningRight = false;
            turningLeft = false;
            approaching = true;
        } else if(!candleFound && flameDetectedRight && !flameDetectedLeft && !turningRight) {
            DEBUG_PRINT("candle detected");
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, -5);
            turningRight = true;
            turningLeft = false;
            approaching = false;
        } else if(!candleFound && flameDetectedLeft && !flameDetectedRight && !turningLeft) {
            DEBUG_PRINT("candle detected");
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, 5);
            turningLeft = true;
            turningRight = false;
            approaching = false;
        }

        if (!candleFound && approaching) {
            //rangefinders.front

        }

        /*
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
        */
    }
    
    void do_explore() {

    }

    void do_move_path() {
        if (MotorControl::current_command.status != CommandStatus::DONE) return;
        if (pathfinder.path.empty()) return;

        // When we've completed our last command pull a new one from the path and execute it
        Move_t next_move;
        pathfinder.path.pop_into(next_move);

        // Never drive into unknown territory
        if (next_move == Move_t::FORWARD) {
            uint8_t next_x, next_y;
            get_front_square(x_pos, y_pos, orientation, next_x, next_y);

            if (pathfinder.map[next_x][next_y].terrain == Terrain::UNKNOWN) {
                    DEBUG_PRINT("Determing next square type");
                    pathfinder.map[next_x][next_y].terrain = colorsensor.curr_terrain;

                    // Recalculate path
                    pathfinder.planPath();

                    Serial.println("Steps: " + String(pathfinder.path.size()));
                    String s = "";
                    for(int8_t i = pathfinder.path.size() - 1; i >= 0; i--)
                        s = s + String(pathfinder.plan[i]) + ",";
                    DEBUG_PRINT(s);
                    pathfinder.path.pop_into(next_move);
            }
        }

        switch(next_move) {
            case Move_t::FORWARD:
                DEBUG_PRINT("Path planning executing move");
                MotorControl::send_command(Command_t::DRIVE, 30);
                get_front_square(x_pos, y_pos, orientation, x_pos, y_pos);
                break;
            case Move_t::TURN_LEFT:
                DEBUG_PRINT("Path planning executing left turn");
                MotorControl::send_command(Command_t::TURN, 90);
                break;
            case Move_t::TURN_RIGHT:
            DEBUG_PRINT("Path planning executing right turn");
                MotorControl::send_command(Command_t::TURN, -90);
                break;
            default:
                break;
        };
    }

    void get_front_square(uint8_t curr_x, uint8_t curr_y, int8_t orientation, uint8_t& x, uint8_t& y) {
        // Assume we are never facing a wall
        // Transform to a positive orientation between 0 and 3
        orientation %= 4;

        switch (orientation) {
            // Along x-axis
            case 0:
                x = curr_x + 1;
                y = curr_y;
                break;
            // Along y-axis
            case 1:
                x = curr_x;
                y = curr_y + 1;
                break;
            case 2:
                x = curr_x - 1;
                y = curr_y;
                break;
            case 3:
                x = curr_x;
                y = curr_y - 1;
                break;
            default:
                break;
        }
    }
    
    void do_explore() {

    }

    void do_move_path() {
        if (MotorControl::current_command.status != CommandStatus::DONE) return;
        if (pathfinder.path.empty()) return;

        // When we've completed our last command pull a new one from the path and execute it
        Move_t next_move;
        pathfinder.path.pop_into(next_move);

        // Never drive into unknown territory
        if (next_move == Move_t::FORWARD) {
            uint8_t next_x, next_y;
            get_front_square(x_pos, y_pos, orientation, next_x, next_y);

            if (pathfinder.map[next_x][next_y].terrain == Terrain::UNKNOWN) {
                    DEBUG_PRINT("Determing next square type");
                    pathfinder.map[next_x][next_y].terrain = colorsensor.curr_terrain;

                    // Recalculate path
                    pathfinder.planPath();

                    Serial.println("Steps: " + String(pathfinder.path.size()));
                    String s = "";
                    for(int8_t i = pathfinder.path.size() - 1; i >= 0; i--)
                        s = s + String(pathfinder.plan[i]) + ",";
                    DEBUG_PRINT(s);
                    pathfinder.path.pop_into(next_move);
            }
        }

        switch(next_move) {
            case Move_t::FORWARD:
                DEBUG_PRINT("Path planning executing move");
                MotorControl::send_command(Command_t::DRIVE, 30);
                get_front_square(x_pos, y_pos, orientation, x_pos, y_pos);
                break;
            case Move_t::TURN_LEFT:
                DEBUG_PRINT("Path planning executing left turn");
                MotorControl::send_command(Command_t::TURN, 90);
                break;
            case Move_t::TURN_RIGHT:
            DEBUG_PRINT("Path planning executing right turn");
                MotorControl::send_command(Command_t::TURN, -90);
                break;
            default:
                break;
        };
    }

    void get_front_square(uint8_t curr_x, uint8_t curr_y, int8_t orientation, uint8_t& x, uint8_t& y) {
        // Assume we are never facing a wall
        // Transform to a positive orientation between 0 and 3
        orientation %= 4;

        switch (orientation) {
            // Along x-axis
            case 0:
                x = curr_x + 1;
                y = curr_y;
                break;
            // Along y-axis
            case 1:
                x = curr_x;
                y = curr_y + 1;
                break;
            case 2:
                x = curr_x - 1;
                y = curr_y;
                break;
            case 3:
                x = curr_x;
                y = curr_y - 1;
                break;
            default:
                break;
        }
    }
};
