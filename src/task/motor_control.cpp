#include "motor_control.h"

// this task = t_motorControl
namespace MotorControl{

etl::queue<Command, 255, etl::memory_model::MEMORY_MODEL_SMALL> command_queue;
Command current_command;

void stopMotors() {
    current_command.type = Command_t::STOP;
    current_command.status = CommandStatus::WAITING;
}

void init_motor_control() {
    current_command.type = Command_t::NONE;
    DEBUG_PRINT("Init motors");

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
    motors.right->init(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_SENSOR_A_PIN, RIGHT_MOTOR_SENSOR_B_PIN, Direction::REVERSE);

    t_motorControl.setCallback(&motor_control);
}
void run_drive_command() {
    if (current_command.status == CommandStatus::DONE) return;

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
    int8_t direction = (current_command.value > 0) ? 1 : -1;
    DEBUG_PRINT(direction);

    uint16_t command_value = current_command.value;

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

    static bool run_init = true;

    uint16_t command_value = current_command.value;

    // 1 is right, -1 is left
    int8_t direction = (current_command.value > 0) ? 1 : -1;
    if (run_init) {
        imu->zero_yaw();
        
        motors.left->setSpeed(50);
        motors.right->setSpeed(-50);
        run_init = false;
    }


    if (abs(motors.left->distance) > 17 && abs(motors.right->distance) > 17) {
        motors.stop();
        motors.left->resetDistance();
        motors.right->resetDistance();

        run_init=true;
        current_command.status = CommandStatus::DONE;
    }
}

void run_stop_command() {
    if (current_command.status == CommandStatus::DONE) return;

    motors.stop();
    current_command.status = CommandStatus::DONE;
}


void run_current_command() {
    if (current_command.status == CommandStatus::DONE) {
        if (!command_queue.empty()) {
            command_queue.pop_into(current_command);
            DEBUG_PRINT("Command recieved");
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
    DEBUG_PRINT("Motor control");
    
    static int count = 0;

    DEBUG_PRINT(motors.left->ticks_) 
    DEBUG_PRINT(motors.left->distance)
    DEBUG_PRINT(motors.right->ticks_)
    DEBUG_PRINT(motors.right->distance)

    static bool done = false;

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

