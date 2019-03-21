#include "motor_control.h"
#include "process_sensors.h"
#include "object_detection.h"
#include "mission_control.h"

// this task = t_motorControl
namespace MotorControl {

etl::queue<Command, 255, etl::memory_model::MEMORY_MODEL_SMALL> command_queue;
Command current_command;

bool stopOnWater=false;

static uint8_t delay_num = 0;

float Kp = 10;//, Ki = 0.05;
int8_t count = 0;
int16_t array[5] = {0};
int16_t integral = 0;
int16_t lastX = 0;

void init_motor_control() {
    DEBUG_PRINT("Init motors");
    current_command.type = Command_t::NONE;
    current_command.status = CommandStatus::DONE;

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
    motors.right->init(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_SENSOR_A_PIN, RIGHT_MOTOR_SENSOR_B_PIN, Direction::REVERSE);

    t_motorControl.setCallback(&motor_control);
}

void stopMotors() {
    current_command.type = Command_t::STOP;
    current_command.status = CommandStatus::WAITING;
}

void setCorrection(){

    int16_t  position         =  0;
    int16_t  error = 0, column = 0;
    //integral = 0;
    
    bool invert = 0;

    // if(MissionControl::orientation == 0){
    //     position = Y;
    //     invert = false;
    // }
    // else if(MissionControl::orientation == 1){
    //     position = X;
    //     invert = true;
    // }
    // else if(MissionControl::orientation == 2){
    //     position = Y;
    //     invert = true;
    // }
    // else{
    //     position = X;
    //     invert = false;
    // }

    position = rangefinders.right.last_reading;

    // error[0] = position -  150;//   0 + 150
    // error[1] = position -  450;// 300 + 150
    // error[2] = position -  750;// 600 + 150
    // error[3] = position - 1050;// 900 + 150
    // error[4] = position - 1350;//1200 + 150
    // error[5] = position - 1650;//1500 + 150

    //column = position/305;
    //error = position - (column * 305 + 152);


    error = position-lastX;
    lastX = position;

    // array[count%5] = error;

    // for(int i = 0; i < 5; i++){
    //     integral = integral + array[i];
    // }

    PLOTTER_SERIAL.print(X);
    PLOTTER_SERIAL.print(",");

    PLOTTER_SERIAL.print(Y);
    PLOTTER_SERIAL.print(",");

    PLOTTER_SERIAL.print(position);
    PLOTTER_SERIAL.print(",");
        
    PLOTTER_SERIAL.print(MissionControl::orientation*100);
    PLOTTER_SERIAL.print(",");

    PLOTTER_SERIAL.print(column*100);
    PLOTTER_SERIAL.print(",");

    PLOTTER_SERIAL.print(error);
    PLOTTER_SERIAL.print(",");

    PLOTTER_SERIAL.print(correction);
    PLOTTER_SERIAL.println(",");

    count++;
    
    //positive error requires negative correction (positive correction = negative error =  turn left)
    if(invert == 0)
        correction = -1 * error * Kp;
    else
        correction = error * Kp;
}

void run_drive_command() {
    if (current_command.status == CommandStatus::DONE) return;

    static bool run_init = true;
    static bool run_ramp[5] = {true, true, true, true, true};
    static int16_t speed_command = 0;

    if (run_init) {
        motors.stop();
        motors.left->resetDistance();
        motors.right->resetDistance();

        stopOnWater = true;
        run_init = false;
        current_command.status = CommandStatus::RUNNING;
    }

    double leftD = motors.left->getDistance();
    double rightD = motors.right->getDistance();
    int8_t direction = (current_command.value > 0) ? 1 : -1;

    uint16_t command_value = current_command.value;

    #ifdef NO_MOTOR_RAMP
    motors.setSpeed(50, correction);
    #else
    if (speed_command == 0) {
        speed_command = 25*direction;
        motors.setSpeed(speed_command, correction);
    }
    else if (abs(speed_command) == 25 && abs(leftD) > abs(command_value)/10 && abs(rightD) > abs(command_value)/10) {
        speed_command = 50*direction;
        motors.setSpeed(speed_command, correction);
    }
    else if (abs(speed_command) == 50 && abs(leftD) > abs(command_value)/5 && abs(rightD) > abs(command_value)/5) {
        speed_command = 75*direction;
        motors.setSpeed(speed_command, correction);
    }
    else if (abs(speed_command) == 75 && abs(leftD) > 0.8*abs(command_value) && abs(rightD) > 0.8*abs(command_value)) {
        speed_command = 50*direction;
        motors.setSpeed(speed_command, correction);
    }
    else if (abs(speed_command) == 50 && abs(leftD) > 0.9*abs(command_value) && abs(rightD) > 0.9*abs(command_value)) {
        speed_command = 25*direction;
        motors.setSpeed(speed_command, correction);
    }
    #endif

    // End, reset
    if (abs(leftD) > abs(command_value) || abs(rightD) > abs(command_value)) {
        DEBUG_PRINT("Done drive section");
        motors.stop();
        motors.left->resetDistance();
        motors.right->resetDistance();

        MissionControl::update_position(command_value);

        run_init = true;
        stopOnWater = false;
        memset(run_ramp,true,sizeof(bool)*5);
        speed_command = 0;
        delay_num = 0;
        current_command.status = CommandStatus::DONE;
    }

    /*
    if (magnetics.magnetDetected) {
        motors.stop();
        command_running = false;
    }
    */
}

void run_turn_command() {
    if (current_command.status == CommandStatus::DONE) return;

    int16_t command_value = current_command.value;

    // 1 is right, -1 is left
    int8_t direction = (current_command.value > 0) ? 1 : -1;
    if (current_command.status == CommandStatus::WAITING) {
        motors.stop();
        
        motors.left->resetDistance();
        motors.right->resetDistance();

        imu->zero_yaw();
        
        // negative direction is right, positive direction in left
        motors.left->setSpeed(direction*-MOTOR_TURN_SPEED);
        motors.right->setSpeed(direction*+MOTOR_TURN_SPEED);
        current_command.status = CommandStatus::RUNNING;
    }

    if (abs(motors.left->distance) > abs((command_value/90)*18.7) && abs(motors.right->distance) > abs((command_value/90)*18.7)) {
        DEBUG_PRINT("Done Turn");
        motors.stop();
        
        motors.left->resetDistance();
        motors.right->resetDistance();

        MissionControl::orientation += command_value/90;

        // C++ modulus operator does not make negative numbers positive
        if (MissionControl::orientation > 3) {
            MissionControl::orientation %= 4;
        } else if (MissionControl::orientation < 0) {
            MissionControl::orientation += 4;
        }
        delay_num = 0;
        current_command.status = CommandStatus::DONE;
        DEBUG_PRINT("Current Orientation")
        DEBUG_PRINT(MissionControl::orientation);
    }
}

void run_stop_command() {
    if (current_command.status == CommandStatus::DONE) return;
    DEBUG_PRINT("STOPPING");

    motors.stop();
    motors.left->resetDistance();
    motors.right->resetDistance();
    
    delay_num = 0;
    current_command.status = CommandStatus::DONE;
    stopOnWater = false;
}

void run_current_command() {
    if (current_command.status == CommandStatus::DONE) {
        if (delay_num < DELAY_COUNT) {
            delay_num += 1;
            return;
        }
        if (!command_queue.empty()) {
            command_queue.pop_into(current_command);
            if (current_command.type == Command_t::DRIVE) {
                DEBUG_PRINT("RUNNING DRIVE COMMAND");
                DEBUG_PRINT(current_command.value);
            }
            else if (current_command.type == Command_t::TURN) { 
                DEBUG_PRINT("RUNNING TURN COMMAND");
                DEBUG_PRINT(current_command.value);
            }

        }
    }
    switch(current_command.type) {
        case Command_t::DRIVE:
            run_drive_command();
            break;
        case Command_t::TURN:
            run_turn_command();
            break;
        case Command_t::STOP:
            run_stop_command();
            break;
        default:
            break;
    };
}
void motor_control() {
    //DEBUG_PRINT("Motor control");
    
    static int count = 0;
    // DEBUG_PRINT(motors.left->ticks_) 
    // DEBUG_PRINT(motors.left->distance)
    // DEBUG_PRINT(motors.right->ticks_)
    // DEBUG_PRINT(motors.right->distance)

    static bool done = false;

    setCorrection();//BB

    run_current_command();

    motors.left->adjustSpeed();
    motors.right->adjustSpeed();

    count++;
}

void send_command(Command_t type, int16_t value) {
    DEBUG_PRINT("Sending command");
    
    Command newCommand = Command{type, value, CommandStatus::WAITING};
    command_queue.push(newCommand);
}

};

