#include "motor_control.h"

// this task = t_motorControl


static Command current_command;
static int16_t command_value;
static bool command_running = false;

void init_motor_control() {
    DEBUG_PRINT("Init motors");

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
    motors.right->init(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_SENSOR_A_PIN, RIGHT_MOTOR_SENSOR_B_PIN, Direction::REVERSE);

    t_motorControl.setCallback(&motor_control);
}
void run_drive_command() {
    DEBUG_PRINT("here");
    if (!command_running) return;
    DEBUG_PRINT("here2");

    static bool run_init = true;
    static bool run_ramp[5] = {true, true, true, true, true};
    static int16_t speed_command = 0;

    if (run_init) {
        motors.left->resetDistance();
        motors.right->resetDistance();

        run_init = false;
    }

    double leftD = motors.left->getDistance();
    double rightD = motors.right->getDistance();
    int8_t direction = (command_value > 0) ? 1 : -1;
    DEBUG_PRINT(direction);

    if (speed_command == 0) {
        speed_command = 25*direction;
        motors.setSpeed(speed_command);
    }
    else if (abs(speed_command) == 25 && abs(leftD) > abs(command_value)/10 && abs(rightD) > abs(command_value)/10) {
        speed_command = 50*direction;
        motors.setSpeed(speed_command);
    }
    else if (abs(speed_command) == 50 && abs(leftD) > abs(command_value)/5 && abs(rightD) > abs(command_value)/5) {
        speed_command = 100*direction;
        motors.setSpeed(speed_command);
    }
    else if (abs(speed_command) == 100 && abs(leftD) > 0.8*abs(command_value) && abs(rightD) > 0.8*abs(command_value)) {
        speed_command = 50*direction;
        motors.setSpeed(speed_command);
    }
    else if (abs(speed_command) == 50 && abs(leftD) > 0.9*abs(command_value) && abs(rightD) > 0.9*abs(command_value)) {
        speed_command = 25*direction;
        motors.setSpeed(speed_command);
    }

    // End, reset
    if (abs(leftD) > abs(command_value) || abs(rightD) > abs(command_value)) {
        motors.stop();
        motors.left->resetDistance();
        motors.right->resetDistance();

        run_init = true;
        memset(run_ramp,true,sizeof(bool)*5);
        speed_command = 0;
        command_running = false;
    }
}
void run_current_command() {
    switch(current_command) {
        case Command::DRIVE:
            run_drive_command();
            break;
        default:
            break;
    };
}
void motor_control() {
    DEBUG_PRINT("Motor control");
    
    static int count = 0;

    DEBUG_PRINT(motors.left->ticks_) 
    DEBUG_PRINT(motors.left->distance)
    DEBUG_PRINT(motors.right->ticks_)
    DEBUG_PRINT(motors.right->distance)

    static bool done = false;
    
    if(!done) {
        current_command = Command::DRIVE;
        command_value = 100;
        command_running = true;
        done = true;
    }

    run_current_command();

    motors.left->adjustSpeed();
    motors.right->adjustSpeed();

    count++;
}

