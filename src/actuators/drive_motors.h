#ifndef MOTORS_H
#define MOTORS_H

#define NUM_COUNTS_PER_REVOLUTION 90

#define LEFT_MOTOR_ENABLE_PIN 12
#define LEFT_MOTOR_PWM_PIN 5

#define RIGHT_MOTOR_ENABLE_PIN 27
#define RIGHT_MOTOR_PWM_PIN 13

#define LEFT_MOTOR_SENSOR_A_PIN 19
#define LEFT_MOTOR_SENSOR_B_PIN 25

#define RIGHT_MOTOR_SENSOR_A_PIN 2
#define RIGHT_MOTOR_SENSOR_B_PIN 10

#define WHEEL_RADIUS 3.81 // cm
#define TICKS_PER_REVOLUTION 208
#define RADS_PER_TICK 0.0302

#define MOTOR_FILTER_ALPHA 0.3
#define MOTOR_CONTROL_CONSTANT 5

#define MAX_MOTOR_SPEED 50 // cm/s

#include <CircularBuffer.h>

#include "common.h"

enum class Direction: int8_t {
    REVERSE = -1,
    FORWARD = 1
};

/*
motor->init({PINS}) must be called first for each motor before setSpeed is used

Wiring:
- Each motor should be wired to the power supply
- Each encoder has 2 hall effect lines, a hall effect ground/5V that need to be wired to the arduino
- The motors PWM and enable (labeled DIR on board) should be wired to arduino. *PWM in code goes to DIR on motor board and enable in code goes to PWM on board*
    - PWM and EN ground should go to arduino ground as well
*/

// Forward declaration so Motors knows that Motor is a class
class Motor;

// Use static class members so we can access them from interupts
class Motors {
    public:

        static Motor *left;
        static Motor *right;

        static void leftMotorInterrupt();
        static void rightMotorInterrupt();

        static void setSpeed(int8_t, int8_t);
        static void stop();
};

class Motor {
    public:
        void init(uint8_t, uint8_t, uint8_t, uint8_t, Direction in_direction=Direction::FORWARD);

        int8_t direction;
        double filtered_speed;

        // Sets motor control parameters
        void setSpeed(int8_t);
        void setSpeed(int8_t, int8_t); // with correction

        // Runs motor control
        void adjustSpeed();

        // Value from -100 to 100 corresponding to motor speed
        // Adjusted to make measured speed equal to set point
        double speed_command;
        // Corresponds to real speed we want
        double speed_setpoint;
        // Feedback from encoders
        double speed;

        void stop();

        // ISR when encoder encounters an edge
        void update();

        void resetDistance();
        double getDistance();
        void readDistance();
        double distance;

        int32_t ticks_;


    private:
        uint8_t get_pwm_command(int8_t);

        Direction orientation_; // If the motor is mounted in forward or reverse

        unsigned long last_;
        uint8_t pwm_pin_;
        uint8_t enable_pin_;
        uint8_t sensor1_pin_;
        uint8_t sensor2_pin_;

        CircularBuffer<double, 100> speeds_;

};

#endif