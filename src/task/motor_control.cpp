#include "motor_control.h"

// this task = t_motorControl

void init_motor_control() {
    DEBUG_PRINT("Init motors");

    motors.left->init(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_SENSOR_A_PIN, LEFT_MOTOR_SENSOR_B_PIN);
    motors.right->init(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_SENSOR_A_PIN, RIGHT_MOTOR_SENSOR_B_PIN, Direction::REVERSE);

    t_motorControl.setCallback(&motor_control);
}

void motor_control() {
    // Just demo code for now
    static int count = 0;

    if (count % 2 == 0) {
        motors.setSpeed(20);
    }
    else {
        motors.setSpeed(-20);
    }
    count++;
}