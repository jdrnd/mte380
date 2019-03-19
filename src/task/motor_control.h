#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>
#include "etl/queue.h"

#include "actuators/drive_motors.h"

#include "mission_control.h"
#include "common.h"

#define MOTOR_PROPORTIONAL_CONTSTANT 5
#define MOTOR_TURN_SPEED 30

#include "sensors/imu.h"
#include "sensors/magnetic.h"

extern Task t_motorControl;
extern Motors motors;

extern Magnetics magnetics;

enum class Command_t: uint8_t {
    DRIVE,
    TURN,
    STOP,
    NONE
};

enum class CommandStatus: uint8_t {
    WAITING = 0,
    RUNNING = 1,
    DONE = 2
};

namespace MotorControl {

    const int DELAY_COUNT = 10;



    struct Command {
        Command_t type;
        int16_t value;
        CommandStatus status;
    };

    extern etl::queue<Command, 255, etl::memory_model::MEMORY_MODEL_SMALL> command_queue; 
    extern Command current_command;
    extern bool stopOnWater;

    void stopMotors();

    void init_motor_control();
    void motor_control();

    void send_command(Command_t, int16_t);
};

#endif