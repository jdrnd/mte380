#include <Arduino.h>
#include <math.h>

#include "motors.h"


Motor* Motors::left = new Motor;
Motor* Motors::right = new Motor;

void Motors::leftMotorInterrupt() {
    left->update();
}

void Motors::rightMotorInterrupt() {
    right->update();
}

void Motors::setSpeed(int8_t speed) {
    Motors::left->setSpeed(speed);
    Motors::right->setSpeed(speed);
}

void Motors::stop() {
    Motors::left->stop();
    Motors::right->stop();
}

// Initializes motor with pins and sets up interrupts
// Must be ran before motor.setSpeed() can be used
void Motor::init(uint8_t pwm_pin, uint8_t enable_pin, uint8_t sensor1_pin, uint8_t sensor2_pin, int8_t in_direction) {
    pwm_pin_ = pwm_pin;
    enable_pin_ = enable_pin;
    sensor1_pin_ = sensor1_pin;
    sensor2_pin_ = sensor2_pin;
    orientation_=in_direction;

    speed=0;

    pinMode(sensor2_pin, INPUT);
    if (sensor1_pin == LEFT_MOTOR_SENSOR_A_PIN) {
        attachInterrupt(digitalPinToInterrupt(sensor1_pin), Motors::leftMotorInterrupt, RISING);
    }
    else if (sensor1_pin == RIGHT_MOTOR_SENSOR_A_PIN) {
        attachInterrupt(digitalPinToInterrupt(sensor1_pin), Motors::rightMotorInterrupt, RISING);
    }
    pinMode(enable_pin_, OUTPUT);

    last_ = micros();
    speeds_.push(0.0);
}

// Accepts value from -20 to +20 corresponding to speed in absolute cm/s
// Should not be used with a value of 0, stop() should be called instead
void Motor::setSpeed(int8_t speedval) {
    double speed_command_ = speedval;

    // scale to [0,255] range
    double pwm_command_ = (speed_command_ + 20) * 255 / 40;

    if (orientation_==REVERSE) {
        pwm_command_ = 255 - pwm_command_;
    }

    digitalWrite(enable_pin_, HIGH);
    analogWrite(pwm_pin_, pwm_command_);
}

// Stops motor immediatly 
void Motor::stop() {
    digitalWrite(enable_pin_, LOW);
}

// Interrupt service routine called when a motor encoder encounters an edge
// Sets speed and direction in the motor class
void Motor::update() {
    if (this == Motors::left) {
        if ( (digitalRead(LEFT_MOTOR_SENSOR_B_PIN) == HIGH && orientation_==REVERSE) || 
                (digitalRead(LEFT_MOTOR_SENSOR_B_PIN) == LOW && orientation_!=REVERSE)) {
            direction = 1;
        }
        else {
            direction = -1;
        }
    }
    else if (this == Motors::right) {
        if ( (digitalRead(RIGHT_MOTOR_SENSOR_B_PIN) == HIGH && orientation_==REVERSE) || 
                (digitalRead(RIGHT_MOTOR_SENSOR_B_PIN) == LOW && orientation_!=REVERSE)) {
            direction = 1;
        }
        else {
            direction = -1;
        }
    }

    unsigned long now = micros();
    unsigned long delta = now - last_;

    // 90 measurements, so 4 degrees per measurement however we only interupt on one rising edge (8 degrees/measurement)
    speed = direction*8/(delta*1.0*10e-6); // currently in degrees/s
    speed = (speed/360)*2*M_PI * WHEEL_RADIUS; // cm/s
    last_ = now;
    speeds_.push(MOTOR_FILTER_ALPHA*speed + (1-MOTOR_FILTER_ALPHA)*speeds_.last());
    filtered_speed = speeds_.last();
}