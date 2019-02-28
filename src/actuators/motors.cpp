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

// Initializes motor with pins and sets up interrupts
// Must be ran before motor.setSpeed() can be used
void Motor::init(uint8_t pwm_pin, uint8_t enable_pin, uint8_t sensor1_pin, uint8_t sensor2_pin, int8_t in_direction) {
    pwm_pin_ = pwm_pin;
    enable_pin_ = enable_pin;
    sensor1_pin_ = sensor1_pin;
    sensor2_pin_ = sensor2_pin;
    orientation_=in_direction;

    pinMode(sensor2_pin, INPUT);
    if (sensor1_pin == LEFT_MOTOR_SENSOR_A_PIN) {
        attachInterrupt(digitalPinToInterrupt(sensor1_pin), Motors::leftMotorInterrupt, RISING);
    }
    else if (sensor1_pin == RIGHT_MOTOR_SENSOR_A_PIN) {
        attachInterrupt(digitalPinToInterrupt(sensor1_pin), Motors::rightMotorInterrupt, RISING);
    }
    pinMode(enable_pin_, OUTPUT);

    last_ = micros();
}

// Accepts value from -100 to +100 corresponding to speed
// Should not be used with a value of 0, stop() should be called instead
void Motor::setSpeed(int8_t speedval) {
    Serial.print(speedval);
    Serial.print(",");
    speedval = map(speedval, -100,100, 0, 255);
    Serial.println((uint8_t)speedval);

    Serial.println(orientation_);
    if (orientation_==REVERSE) {
        speedval = 255 - speedval;
    }

    digitalWrite(enable_pin_, HIGH);
    analogWrite(pwm_pin_, speedval);
}

// Stops motor immediatly 
void Motor::stop() {
    digitalWrite(enable_pin_, LOW);
}

// Interrupt service routine called when a motor encoder encounters an edge
// Sets speed and direction in the motor class
void Motor::update() {

    if (digitalRead(LEFT_MOTOR_SENSOR_B_PIN) == HIGH) {
        //Serial.println("Forwards");
        direction = 1;
    }
    else {
        //Serial.println("Backwards");
        direction = -1;
    }

    unsigned long now = micros();
    unsigned long delta = now - last_;

    // 90 measurements, so 4 degrees per measurement however we only interupt on one rising edge (8 degrees/measurement)
    speed = 8/(delta*1.0*10e-6);
    //Serial.println((int)speed); // currently in degrees/s
    last_ = now;
}




