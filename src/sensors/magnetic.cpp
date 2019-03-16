#include <Arduino.h>

#include "magnetic.h"


const uint8_t Magnetics::sensor_pins_[NUM_MAGNETIC_SENSORS] = {
    A0,
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    A7,
    A8
};

/*
bool Magnetic::detectMagnet() {
    return (abs(analogRead(sensor_pin_) - HALL_EFFECT_MEAN) > HALL_EFFECT_TOL);
}

Magnetics::Magnetics() {
    for (uint8_t i=0; i<sizeof(MAGNETIC_PINS)/sizeof(uint8_t); i++) {
        DEBUG_PRINT(MAGNETIC_PINS[i]);
        Magnetics::sensors_[i].sensor_pin_ = MAGNETIC_PINS[i];
    }
}


*/

void Magnetics::logReadings() {
    for (uint8_t i=0; i<sizeof(MAGNETIC_PINS)/sizeof(uint8_t); i++) {
        PLOTTER_SERIAL.print(Magnetics::sensor_pins_[i]);
        PLOTTER_SERIAL.print(",");
        PLOTTER_SERIAL.print(analogRead(Magnetics::sensor_pins_[i]));
        PLOTTER_SERIAL.print(",");
    }
    PLOTTER_SERIAL.println("0");
}

void Magnetics::detectMagnet() {
    for (uint8_t i=0; i<NUM_MAGNETIC_SENSORS; i++){//sizeof(MAGNETIC_PINS)/sizeof(uint8_t); i++) {
        if (abs(analogRead(Magnetics::sensor_pins_[i])-HALL_EFFECT_MEAN) > HALL_EFFECT_TOL) magnetDetected = true;
    }
}

void Magnetics::clearMagnetDetection() {
    magnetDetected = false;
}