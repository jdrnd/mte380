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

static const int8_t NUM_FINDERS = 2;
static int8_t triggers[NUM_FINDERS];
static const int8_t TRIGGERS[3] = {-15, 5, 10};
static uint8_t finders[NUM_FINDERS];

namespace MissionControl {
    // this task = t_missionControl

    State_t state = STARTING_STATE;
    static uint64_t count = 0;

    bool magnet_found = false;
    Terrain map[6][6];

    //TODO: integrate x_pos and y_pos with X and Y 
    uint8_t x_pos = STARTING_X_POS;
    uint8_t y_pos = STARTING_Y_POS;
    int8_t orientation = STARTING_ORIENTATION;

    bool relocalized = false;

    uint8_t sand_pit_order[NUM_SAND_POSITIONS];
    uint8_t scan_position_order[NUM_SCAN_POSITIONS];

    PathFinder pathfinder;

    void init() {
        DEBUG_PRINT("Init mission control");

        //MotorControl::send_command(Command_t::SLOW_TURN, 110);

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
                break;
            case State_t::OBJECT_SEARCH:
                do_object_search();
                break;
            default:
                break;
        };
        do_relocalize();
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

    bool init_relocalize() {
        if (MotorControl::current_command.type != Command_t::SLOW_TURN)
            return false;
        if (MotorControl::current_command.value > 0)
            orientation = (orientation + 1) % 4;
        else if(MotorControl::current_command.value < 0) {
            if (orientation == 0)
                orientation = 3;
            else
                orientation--;
        }

        uint8_t closest_x = min(x_pos, 5 - x_pos);
        triggers[0] = TRIGGERS[closest_x];
        uint8_t closest_y = min(y_pos, 5 - y_pos);
        triggers[1] = TRIGGERS[closest_y];

        if (x_pos < 3) {
            /*
            0 -> 3
            1 -> 2
            2 -> 1
            3 -> 0
            */
            finders[0] = 3 - orientation;
        } else {
            /*
            0 -> 1
            1 -> 0
            2 -> 3
            3 -> 2
            */
            finders[0] = 2 * (orientation / 2) + (orientation + 1) % 2;
        }

        if (y_pos < 3) {
            /*
            0 -> 0
            1 -> 3
            2 -> 2
            3 -> 1
            */
            finders[1] = (orientation + 2 * (orientation % 2)) % 4;
        } else {
            /*
            0 -> 2
            1 -> 1
            2 -> 0
            3 -> 3
            */
            finders[1] = (orientation + 2 * ((orientation + 1) % 2)) % 4;
        }

        Serial.println(
            "\nx_pos: " + String(x_pos) + 
            "\ny_pos: " + String(y_pos) + 
            "\norientation: " + String(orientation)
        );

        Serial.println(
            "\ntrigger0: " + String(triggers[0]) +
            "\ntrigger1: " + String(triggers[1]) +
            "\nfinders0: " + String(finders[0]) +
            "\nfinders1: " + String(finders[1])
        );

        return true;
    }

    void do_relocalize() {
        const uint8_t DIFF_SIZE = 1;
        const uint8_t DELAY_CHECK = 20;
        static uint8_t readingData = 0;
        static bool initialized = false;
        /* TRIGGER NOTES with diff size 3: 
            don't use this: 18 - best for lidar that are 2 tiles away from the wall this was with 45 DELAY_CHECK'
            10 - best for lidar that are 2 tiles away from the wall this was with 25 DELAY_CHECK
            5 - best for lidar that is 1 tile away from from the wall with 25 DELAY_CHECK
            try -15 for 0 tiles away
        */
        static int8_t index = 0;
        // static uint8_t delay_stop = 0;

        static int16_t diffs[DIFF_SIZE][NUM_FINDERS];
        static uint16_t prev_value[NUM_FINDERS];
        static int16_t sum[NUM_FINDERS];
        static int16_t prev_sum[NUM_FINDERS];
        static bool finder_triggered[NUM_FINDERS];

        if (!initialized) {
            if (init_relocalize()) {
                initialized = true;
                relocalized = false;
                finder_triggered[0] = false;
                finder_triggered[1] = false;
                readingData = 0;
                index = 0;
            } else
                return;
        }

        if (readingData == 0) {
            readingData++;
        } else if (readingData > 0) {
            diffs[index][0] = rangefinders[finders[0]].readings_.last() - prev_value[0];
            diffs[index][1] = rangefinders[finders[1]].readings_.last() - prev_value[1];

            sum[0] = 0;
            sum[1] = 0;
            for(size_t i = 0; i < DIFF_SIZE; i++) {
                sum[0] += diffs[i][0];
                sum[1] += diffs[i][1];
            }

            if (sum[0] > -triggers[0] && prev_sum[0] <= -triggers[0])
                finder_triggered[0] = true;
            if (sum[1] > -triggers[1] && prev_sum[1] <= -triggers[1])
                finder_triggered[1] = true;

            if (readingData == DELAY_CHECK && (
                finder_triggered[0] && finder_triggered[1]
            )) {
                relocalized = true;
            }
            /*
            PLOTTER_SERIAL.println(
                String(readingData * 5) + "," + 
                String(rangefinders[finders[0]].last_reading) + "," + 
                String(prev_value[0]) + "," + 
                String(diffs[index][0]) + ", " +
                String(sum[0]) + ", " +
                String(rangefinders[finders[1]].last_reading + 2000) + "," + 
                String(prev_value[1] + 2000) + "," + 
                String(diffs[index][1] + 2000) + ", " +
                String(sum[1] + 2000) + ", "
            );
            */

            index = (index + 1) % DIFF_SIZE;
            prev_sum[0] = sum[0];
            prev_sum[1] = sum[1];
            if (readingData < DELAY_CHECK)
                readingData++;
        }
        prev_value[0] = rangefinders[finders[0]].readings_.last();
        prev_value[1] = rangefinders[finders[1]].readings_.last();
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
            init_damper();
            raise_damper();
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

            if (curr_scan_position == NUM_SCAN_POSITIONS) {
                MotorControl::send_command(Command_t::TURN, 360);
            }
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

    void do_object_search() {
        static bool done_init = false;

        static bool first_planned = false;
        static bool second_planned = false;
        static bool return_planned = false;

        if (!done_init) {
            MotorControl::send_command(Command_t::DRIVE, 120);
            done_init = true;
        }

        if (MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE && !first_planned) {
            ObjectDetection::find_best_points();
            
            ObjectDetection::print_object_data();

            DEBUG_PRINT(ObjectDetection::points[0].x)
            DEBUG_PRINT(ObjectDetection::points[0].y)
            DEBUG_PRINT(ObjectDetection::points[1].x)
            DEBUG_PRINT(ObjectDetection::points[1].y)
            DEBUG_PRINT(ObjectDetection::points[2].x)
            DEBUG_PRINT(ObjectDetection::points[2].y)
            pathfinder.setBotPosition(x_pos, y_pos, orientation);
            pathfinder.setTargetPosition(ObjectDetection::points[0].x, ObjectDetection::points[0].y);
            pathfinder.planPath();

            // Don't send last move as that would cause us to run into the object
            while(pathfinder.path.size() > 1) send_next_planned_move();
            if (pathfinder.path.top().type == Move_t::FORWARD) {
                if (pathfinder.path.top().value > 30) {
                    MotorControl::send_command(Command_t::DRIVE, pathfinder.path.top().value -30);
                }
            }
            first_planned = true; 
        }

        if (MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE && first_planned && !second_planned) {
            init_damper();
            raise_damper();
            count=0;
            pathfinder.setBotPosition(x_pos, y_pos, orientation);
            pathfinder.setTargetPosition(ObjectDetection::points[1].x, ObjectDetection::points[1].y);

            pathfinder.planPath();

            // Don't send last move as that would cause us to run into the object
            while(pathfinder.path.size() > 1) send_next_planned_move();
                        if (pathfinder.path.top().type == Move_t::FORWARD) {
                if (pathfinder.path.top().value > 30) {
                    MotorControl::send_command(Command_t::DRIVE, pathfinder.path.top().value -30);
                }
            }
            second_planned = true;
        }

        // return to start
        if (MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE &&first_planned && second_planned && !return_planned) {
            init_damper();
            raise_damper();
            count=0;
            pathfinder.setBotPosition(x_pos, y_pos, orientation);
            pathfinder.setTargetPosition(STARTING_X_POS, STARTING_Y_POS);

            pathfinder.planPath();

            while(pathfinder.path.size() > 0) send_next_planned_move();
            return_planned = true;
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
            magnet_found = true;
            DEBUG_PRINT("Magnet detected");
            state = State_t::NONE;
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
