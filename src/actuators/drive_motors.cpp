#include <Arduino.h>
#include <math.h>

#include "drive_motors.h"


Motor* Motors::left = new Motor;
Motor* Motors::right = new Motor;

void Motors::leftMotorInterrupt() {
    left->update();
}

void Motors::rightMotorInterrupt() {
    right->update();
}

void Motors::setSpeed(int8_t speed, int8_t correction1) {
    if (correction1 > 100)
        correction1 = 100;
    else if (correction1 < - 100)
        correction1 = -100;

    if (correction1 > 0){
        Motors::left->setSpeed((int8_t)(speed * (100 - correction1) / 100.));
        Motors::right->setSpeed(speed);
    } else {
        Motors::left->setSpeed(speed);
        Motors::right->setSpeed((int8_t)(speed * (100 + correction1) / 100.));
    }
        
        // PLOTTER_SERIAL.print(speed);
        // PLOTTER_SERIAL.print(",");
        
        // PLOTTER_SERIAL.print(speed * (100 - correction1) / 100.);
        // PLOTTER_SERIAL.print(",");
    
        // PLOTTER_SERIAL.print(correction1);
        // PLOTTER_SERIAL.println(",");
}

void Motors::stop() {
    Motors::left->stop();
    Motors::right->stop();
}

// Initializes motor with pins and sets up interrupts
// Must be ran before motor.setSpeed() can be used
void Motor::init(uint8_t pwm_pin, uint8_t enable_pin, uint8_t sensor1_pin, uint8_t sensor2_pin, Direction in_direction) {
    pwm_pin_ = pwm_pin;
    enable_pin_ = enable_pin;
    sensor1_pin_ = sensor1_pin;
    sensor2_pin_ = sensor2_pin;
    orientation_=in_direction;
    ticks_ = 0;

    speed=0;
    speed_setpoint=0;
    speed_command=0;

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

// Accepts value from +100 to -100 corresponding to full speed forward and full speed reverse
// Should not be used with a value of 0, stop() should be called instead
void Motor::setSpeed(int8_t speedval) {
    speed_setpoint = (speedval*1.0 / 100.0) * MAX_MOTOR_SPEED;
    speed_command = speedval;

    digitalWrite(enable_pin_, HIGH);
}

void Motor::adjustSpeed() {
    #ifdef LOG_MOTOR_CONTROL
    /*
    PLOTTER_SERIAL.print(Motors::left->speed_setpoint);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(Motors::left->speed);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(Motors::left->speed_command);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(Motors::right->speed_setpoint);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(Motors::right->speed);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.println(Motors::right->speed_command);
    */
    PLOTTER_SERIAL.print(Motors::left->distance);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.println(Motors::right->distance);
    #endif

    // Vary speed_command until speed is close to speed_setpoint
    if (speed_setpoint > 0) {
        if (speed > speed_setpoint && (speed-speed_setpoint) > 1){
            speed_command -= MOTOR_CONTROL_CONSTANT;
            if (speed_command < 0) speed_command = 0;
        }
        else if(speed < speed_setpoint && (speed_setpoint-speed) > 1) {
            speed_command += MOTOR_CONTROL_CONSTANT;
        }
    }
    else if (speed_setpoint < 0) {
        if (speed < speed_setpoint && (speed_setpoint-speed) > 1) {
            speed_command += MOTOR_CONTROL_CONSTANT;
            if (speed_command > 0) speed_command = 0;
        }
        else if (speed > speed_setpoint && (speed-speed_setpoint) > 1) {
            speed_command -= MOTOR_CONTROL_CONSTANT;
        }
    }

    // Ensure we never go outside motor control bounds
    if (speed_command > 100) speed_command = 100;
    if (speed_command < -100) speed_command = -100;
    
    uint8_t pwm_command = get_pwm_command(speed_command);

    #ifdef LOG_MOTOR_CONTROL
    //DEBUG_PRINT(pwm_command);
    #endif
    
    analogWrite(pwm_pin_, pwm_command);
}

// Scale from -100 to 100 range to 0-255, account for motor direction
uint8_t Motor::get_pwm_command(int8_t speed_command_val) {

    // scale to [0,255] range
    int32_t pwm_command = map(speed_command, -100, 100, 0, 255);

    if (orientation_==Direction::REVERSE) {
        pwm_command = 255 - pwm_command;
    }
    return (uint8_t)pwm_command;
}


// Stops motor immediatly 
void Motor::stop() {
    digitalWrite(enable_pin_, LOW);
    speed_command = 0;
    speed_setpoint = 0;
}

// Interrupt service routine called when a motor encoder encounters an edge
// Sets speed and direction in the motor class
void Motor::update() {
    if (this == Motors::left) {
        if ( (digitalRead(LEFT_MOTOR_SENSOR_B_PIN) == HIGH && orientation_==Direction::REVERSE) || 
                (digitalRead(LEFT_MOTOR_SENSOR_B_PIN) == LOW && orientation_!=Direction::REVERSE)) {
            direction = 1;
        }
        else {
            direction = -1;
        }
    }
    else if (this == Motors::right) {
        if ( (digitalRead(RIGHT_MOTOR_SENSOR_B_PIN) == HIGH && orientation_==Direction::REVERSE) || 
                (digitalRead(RIGHT_MOTOR_SENSOR_B_PIN) == LOW && orientation_!=Direction::REVERSE)) {
            direction = 1;
        }
        else {
            direction = -1;
        }
    }
    ticks_ += direction;

    unsigned long now = micros();
    unsigned long delta = now - last_;

    // 90 measurements, so 4 degrees per measurement however we only interupt on one rising edge (8 degrees/measurement)
    speed = 10*direction*RADS_PER_TICK*WHEEL_RADIUS/(delta*1.0*10e-6);
    last_ = now;
    speeds_.push(MOTOR_FILTER_ALPHA*speed + (1-MOTOR_FILTER_ALPHA)*speeds_.last());
    filtered_speed = speeds_.last();
}

void Motor::resetDistance() {
    ticks_ = 0;
}

double Motor::getDistance() {
    return (ticks_*RADS_PER_TICK*WHEEL_RADIUS);
}

void Motor::readDistance() {
    distance = getDistance();
}