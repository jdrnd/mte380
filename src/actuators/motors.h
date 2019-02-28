#ifndef MOTORS_H
#define MOTORS_H

#define NUM_COUNTS_PER_REVOLUTION 90

#define LEFT_MOTOR_SENSOR_A_PIN 2
#define LEFT_MOTOR_SENSOR_B_PIN 10

#define LEFT_MOTOR_ENABLE_PIN 27
#define LEFT_MOTOR_PWM_PIN 13

#define RIGHT_MOTOR_ENABLE_PIN 12
#define RIGHT_MOTOR_PWM_PIN 5

// TODO set these
#define RIGHT_MOTOR_SENSOR_A_PIN 0
#define RIGHT_MOTOR_SENSOR_B_PIN 0

enum : int8_t {
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
};

class Motor {
    public:
        void init(uint8_t, uint8_t, uint8_t, uint8_t, int8_t in_direction=FORWARD);

        int8_t direction;
        double speed;

        void setSpeed(int8_t);
        void stop();

        // ISR when encoder encounters an edge
        void update();
    private:
        int8_t orientation_;

        unsigned long last_;
        uint8_t pwm_pin_;
        uint8_t enable_pin_;
        uint8_t sensor1_pin_;
        uint8_t sensor2_pin_;
};

#endif