#include "motor_control.h"
#include "process_sensors.h"

// this task = t_motorControl
namespace MotorControl {

etl::queue<Command, 255, etl::memory_model::MEMORY_MODEL_SMALL> command_queue;
Command current_command;

bool stopOnWater=false;

static uint8_t delay_num = 0;

int8_t correction = 0;

int8_t Kp = 20;

int16_t leftValArray[5] = {0};//BB
int8_t counter = 0;//BB

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
    //differentiate left sensor value and use as correction
    
    int16_t slopeSum = 0;

    leftValArray[counter] = LEFTLIDARVAL;

    counter++;

    if(counter == 5){
        for(int i = 0; i < 4; i++){
            slopeSum = slopeSum + leftValArray[i+1] - leftValArray[i];
        }

        correction = slopeSum / 5 * Kp;

        counter = 0;

        for(int i = 0; i < 5; i++){
            leftValArray[i] = 0;
        }
    }
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
    }

    double leftD = motors.left->getDistance();
    double rightD = motors.right->getDistance();
    int8_t direction = (current_command.value > 0) ? 1 : -1;
    DEBUG_PRINT(direction);

    uint16_t command_value = current_command.value;

    motors.setSpeed(50, correction);


        // PLOTTER_SERIAL.print(correction);
        // PLOTTER_SERIAL.println(",");

    // if (speed_command == 0) {
    //     speed_command = 25*direction;
    //     motors.setSpeed(speed_command, correction);
    // }
    // else if (abs(speed_command) == 25 && abs(leftD) > abs(command_value)/10 && abs(rightD) > abs(command_value)/10) {
    //     speed_command = 50*direction;
    //     motors.setSpeed(speed_command, correction);
    // }
    // else if (abs(speed_command) == 50 && abs(leftD) > abs(command_value)/5 && abs(rightD) > abs(command_value)/5) {
    //     speed_command = 75*direction;
    //     motors.setSpeed(speed_command, correction);
    // }
    // else if (abs(speed_command) == 75 && abs(leftD) > 0.8*abs(command_value) && abs(rightD) > 0.8*abs(command_value)) {
    //     speed_command = 50*direction;
    //     motors.setSpeed(speed_command, correction);
    // }
    // else if (abs(speed_command) == 50 && abs(leftD) > 0.9*abs(command_value) && abs(rightD) > 0.9*abs(command_value)) {
    //     speed_command = 25*direction;
    //     motors.setSpeed(speed_command, correction);
    // }

    // End, reset
    if (abs(leftD) > abs(command_value) || abs(rightD) > abs(command_value)) {
        motors.stop();
        motors.left->resetDistance();
        motors.right->resetDistance();

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
    DEBUG_PRINT("RUNNING TURN");

    int16_t command_value = current_command.value;

    // 1 is right, -1 is left
    int8_t direction = (current_command.value > 0) ? 1 : -1;
    if (current_command.status == CommandStatus::WAITING) {
        DEBUG_PRINT("RUNNING TURN INIT");

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
        DEBUG_PRINT("doneturn");
        motors.stop();
        
        motors.left->resetDistance();
        motors.right->resetDistance();

        MissionControl::orientation += command_value/90;
        delay_num = 0;
        current_command.status = CommandStatus::DONE;
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

