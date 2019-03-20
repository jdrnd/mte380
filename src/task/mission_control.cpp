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
                do_candle_homing2();
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


    void do_candle_homing2() {
        static bool candleFound = false;
        // static bool scanned = false;
        // static bool scanning = false;
        static uint8_t flameState = 0;

        bool fdbr = flameDetectedBottomRight;
        bool fdbl = flameDetectedBottomLeft;
        bool fdtr = flameDetectedTopRight;
        bool fdtl = flameDetectedTopLeft;

        static double sum = 0;
        static uint16_t count = 0;

        //static double commanded = 0;
        //static double offset = 0;
        static double distanceToCandle = 0;

        bool still = MotorControl::current_command.status == CommandStatus::DONE && MotorControl::command_queue.empty();

        //PLOTTER_SERIAL.println(String(fdbr) + "," + String(fdbl + 2) + "," + String(fdtr + 4) + "," + String(fdtl + 6));

        //PLOTTER_SERIAL.println(String(flameState));
        PLOTTER_SERIAL.println("flameState: " + String(flameState) + " still: " + String(still));

        const uint8_t sweep = 90;
        
        if (flameState == 0 && still) {
            MotorControl::send_command(Command_t::FINE_TURN, -20);
            flameState = 1;
        } else if (flameState == 1 && still) {
            MotorControl::send_command(Command_t::SLOW_FINE_TURN, sweep);
            flameState = 2;
        } else if (flameState == 2) {
            if (rangefinders.front.last_reading < 330) {
                sum += MotorControl::getDegrees();
                count++;
            }
            //PLOTTER_SERIAL.println(String(MotorControl::getDegrees()));
            /*PLOTTER_SERIAL.println("sum: " + String(sum) + " count: " + String(count) 
                + " average: " + String(sum / ((double)count))
                + " value: " + String(sweep - sum / ((double)count)));*/
            if (still) {
                double commanded = (sweep - sum / ((double)count));
                commanded = commanded * (1 + 0.6 * exp(- 4.5 * commanded / sweep));
                //PLOTTER_SERIAL.println("sent: " + String(commanded));
                MotorControl::send_command(Command_t::FINE_TURN, - commanded);
                flameState = 3;
            }
        } /*else if (flameState == 3 && still) {
            // TODO @JordanSlater implement an error here if the lidar is greater than some value
            double offset = 1.5 * (180 / PI) * atan(sweep / (rangefinders.front.last_reading + 100.0));
            MotorControl::send_command(Command_t::FINE_TURN, - offset);
            flameState = 4;
        } else if (flameState == 4 && still) {
            MotorControl::send_command(Command_t::SLOW_DRIVE, 100);
            flameState = 5;
        } else if (flameState == 5 && rangefinders.shortrange.last_reading < 100) {
            distanceToCandle = MotorControl::getDistance();
            MotorControl::stopMotors();
            flameState = 6;
        } /*else if (flameState == 6 && still) {
            // do the damping stuff
            flameState = 8;
        } else if (flameState == 8) {
            MotorControl::send_command(Command_t::SLOW_DRIVE, - distanceToCandle);
            flameState = 9;
        }*/
    }

    void do_candle_homing() {
        DEBUG_PRINT("Candle homing")
        static bool turningLeft = false;
        static bool turningRight = false;
        static bool approaching;
        static bool candleFound = false;

        static bool positioning_done = false;

        if (!candleFound) {

        if(MotorControl::command_queue.empty() && MotorControl::current_command.status == CommandStatus::DONE) {
            if (turningRight) turningRight = false;
            if (turningLeft) turningLeft = false;
            if (approaching) approaching = false;
        }

        PLOTTER_SERIAL.println("approaching: " + String(approaching) + 
            " L: " + String(flameDetectedLeft) + " R: " + String(flameDetectedRight) + " candleFound: " + String(candleFound));
        // if candle has not been found and a sensor on each side is triggered
        // advance to the candle
        if(flameDetectedLeft && flameDetectedRight && !approaching) {
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::DRIVE, 10);
            turningRight = false;
            turningLeft = false;
            approaching = true;
        // if a right sensor is triggered turn right
        } else if(flameDetectedRight && !flameDetectedLeft && !turningRight) {
            DEBUG_PRINT("candle detected");
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, -500);
            turningRight = true;
            turningLeft = false;
            approaching = false;
        // if a left sensor is triggered turn left
        } else if(flameDetectedLeft && !flameDetectedRight && !turningLeft) {
            DEBUG_PRINT("candle detected");
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            MotorControl::send_command(Command_t::TURN, 500);
            turningLeft = true;
            turningRight = false;
            approaching = false;
        }

        // if the right sensor is not triggered while turing right stop turning right
        if (turningRight && !flameDetectedRight) {
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            turningRight = false;
        // if the left sensor is not triggered while turing left stop turning left
        } else if (turningLeft && !flameDetectedLeft) {
            MotorControl::stopMotors();
            MotorControl::command_queue.clear();
            turningLeft = false;
        }

        if (approaching) {
            if (rangefinders.shortrange.last_reading < 100) {
                //MotorControl::stopMotors();
                //candleFound = true;
            }
        }

        } // CandleFound
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
