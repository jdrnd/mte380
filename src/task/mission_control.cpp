#include "mission_control.h"

const Position sandpits[NUM_SAND_POSITIONS] = {
    Position{1,4,false},
    Position{2,2,false},
    Position{4,1, false}
};

const Position scan_positions[NUM_SCAN_POSITIONS] = {
    Position{2,1, false},
    Position{1,3,false},
    Position{3,4,false},
    Position{4,2,false}
};

namespace MissionControl {
    // this task = t_missionControl

    State_t state = State_t::TEST_MOVE;
    static uint64_t count = 0;

    bool magnet_found = false;
    Terrain map[6][6];

    //TODO: integrate x_pos and y_pos with X and Y 
    uint8_t x_pos = STARTING_X_POS;
    uint8_t y_pos = STARTING_Y_POS;
    int8_t orientation = STARTING_ORIENTATION;

    uint8_t sand_pit_order[NUM_SAND_POSITIONS];
    uint8_t scan_position_order[NUM_SCAN_POSITIONS];

    PathFinder pathfinder;

    void init() {
        DEBUG_PRINT("Init mission control");
        pathfinder.init();

        // Visit sand pits and candle scan positions in the most efficient order
        switch(STARTING_ORIENTATION) {
            case 0:
                sand_pit_order[0] = 0; sand_pit_order[1] = 1; sand_pit_order[2] = 2;
                scan_position_order[0] = 1, scan_position_order[1] = 2, scan_position_order[2] = 3, scan_position_order[3] = 0;
                break;
            case 1:
                sand_pit_order[0] = 2; sand_pit_order[1] = 1; sand_pit_order[2] = 0; 
                scan_position_order[0] = 0, scan_position_order[1] = 3, scan_position_order[2] = 2, scan_position_order[3] = 1;
                break;
            case 2:
                sand_pit_order[0] = 2; sand_pit_order[1] = 1; sand_pit_order[2] = 0; 
                scan_position_order[0] = 3, scan_position_order[1] = 2, scan_position_order[2] = 1, scan_position_order[3] = 0;
                break;
            case 3:
                sand_pit_order[0] = 0; sand_pit_order[1] = 1; sand_pit_order[2] = 2;
                scan_position_order[0] = 2, scan_position_order[1] = 3, scan_position_order[2] = 0, scan_position_order[3] = 1; 
                break;
        };

        t_missionControl.setCallback(&run);
    }

    void run() {
        switch(state) {
            case State_t::CANDLE_HOMING:
                do_candle_homing();
                break;
            case State_t::CANDLE_SEARCH:
                do_candle_search();
                break;
            case State_t::MOVE:
                do_move_path();
                break;
            case State_t::TEST_MOVE:
                do_test_move();
                break;
            case State_t::FIND_MAGNET:
                do_find_magnet();
            default:
                break;
        };
        count++;

        if (count == 6) {
            fold_damper();
        }
        if (count == 12) {
            deinit_damper();
            digitalWrite(FAN_CONTROL_PIN, LOW);
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

        DEBUG_PRINT("approaching: " + String(approaching) + " L: " + String(flameDetectedLeft) + " R: " + String(flameDetectedRight));
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

    void do_candle_search() {
        static uint8_t curr_scan_position = 0;
        if (curr_scan_position >= NUM_SCAN_POSITIONS) return;

        if (flameDetected) {
            pinMode(FAN_CONTROL_PIN, OUTPUT);
            digitalWrite(FAN_CONTROL_PIN, HIGH);
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            count=0;
            state = State_t::NONE;
            return;
        }

        if (curr_scan_position == NUM_SCAN_POSITIONS) {
            MotorControl::send_command(Command_t::TURN, 360);
        }

        // Send coordinates and plan path to each sand pit
        if (MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {

            if (curr_scan_position == NUM_SCAN_POSITIONS) {
                state = State_t::NONE;
                return;
            }
            pathfinder.setBotPosition(x_pos, y_pos, orientation);
            pathfinder.setTargetPosition(scan_positions[scan_position_order[curr_scan_position]].x, scan_positions[scan_position_order[curr_scan_position]].y);
            pathfinder.planPath();

            if (curr_scan_position != 0) {

                MotorControl::send_command(Command_t::TURN, 360);
            }

            while (!pathfinder.path.empty()) send_next_planned_move();
            curr_scan_position++;

        }
    }

    void do_move_path() {

        static bool done_init = false;
        if (!done_init) {
            MotorControl::send_command(Command_t::TURN, -90);
            MotorControl::send_command(Command_t::TURN, -90);
            MotorControl::send_command(Command_t::TURN, 90);
            MotorControl::send_command(Command_t::TURN, 90);
            done_init = true;
        }
    }

    void do_test_move() {
        static bool done_init = false;

        if (!done_init) {
            MotorControl::send_command(Command_t::DRIVE, 120);
            MotorControl::send_command(Command_t::TURN, 90);
            MotorControl::send_command(Command_t::DRIVE, 90);
            done_init = true;
        }
    }

    void do_find_magnet(){
        if (Magnetics::magnetDetected) {
            #ifdef STOP_ON_MAGNET
            // Signal physically somehow
            magnet_found = true;
            DEBUG_PRINT("Magnet detected");
            state = State_t::CANDLE_HOMING;
            init_damper();
            raise_damper();
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            count = 0;
            return;
            #endif
        }

        static uint8_t curr_sand_pit = 0;
        if (curr_sand_pit >= NUM_SAND_POSITIONS) return;

        // Send coordinates and plan path to each sand pit
        if (MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {
            pathfinder.setBotPosition(x_pos, y_pos, orientation);
            pathfinder.setTargetPosition(sandpits[sand_pit_order[curr_sand_pit]].x, sandpits[sand_pit_order[curr_sand_pit]].y);
            pathfinder.planPath();

            while (!pathfinder.path.empty()) send_next_planned_move();
            curr_sand_pit++;
        }
    }

    // Must ensure that there is a move in the stack before calling
    void send_next_planned_move() {
        // When we've completed our last command pull a new one from the path and execute it
        Move next_move;
        pathfinder.path.pop_into(next_move);

        switch(next_move.type) {
            case FORWARD:
                DEBUG_PRINT("Path planning executing move");
                MotorControl::send_command(Command_t::DRIVE, next_move.value);
                break;
            case TURN_LEFT:
                DEBUG_PRINT("Path planning executing left turn");
                MotorControl::send_command(Command_t::TURN, 90);
                break;
            case TURN_RIGHT:
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

    // Given the last command value and known orientation, set the new robot position
    void update_position(int16_t command_value) {
        int8_t num_tiles = command_value / 30;

        switch(orientation) {
            case 0:
                x_pos += num_tiles;
                break;
            case 1:
                y_pos += num_tiles;
                break;
            case 2:
                x_pos -= num_tiles;
                break;
            case 3:
                y_pos -= num_tiles;
                break;
        }
        DEBUG_PRINT("New position " + String(x_pos) + "," + String(y_pos));
    }
};
