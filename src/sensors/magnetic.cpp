#include <Arduino.h>

#include "magnetic.h"

// Declarations of static class members still need definitions??
Magnetic Magnetics::sensors_[NUM_MAGNETIC_SENSORS];

bool Magnetic::detectMagnet() {
    return (abs(analogRead(sensor_pin_) - HALL_EFFECT_MEAN) > HALL_EFFECT_TOL);
}

Magnetics::Magnetics() {
    for (uint8_t i=0; i<sizeof(sensor_pins)/sizeof(Magnetic); i++) {
        sensors_[i].sensor_pin_ = sensor_pins_[i];
    }
}

bool Magnetics::detectMagnet() {
    for (uint8_t i=0; i<sizeof(sensor_pins)/sizeof(Magnetic); i++) {
        if (sensors_[i].detectMagnet()) return true;
    }
    return false;
}