#ifndef MAGNETICS_H
#define MAGNETICS_H

#include <Arduino.h>

#include "common.h"

/*
    TODO: Mount sensors on robot and determine pinout+tolerancing
    Usable pins: A0-A14 on mega, harder but possible on teensy
    Overall usage: Magnetics::detectMagnet() returns whether or not a magnet
    was detected on any sensor
*/

#define HALL_EFFECT_TOL 6
#define HALL_EFFECT_MEAN 510 // experimentally measured

// Modify here to change number of sensors
// And in magnetics class to add sensor pins
#define NUM_MAGNETIC_SENSORS 9
// Modify in CPP file
extern uint8_t MAGNETIC_PINS[NUM_MAGNETIC_SENSORS];

class Magnetics;

class Magnetics {
    public:
        void detectMagnet();
        void logReadings();
        void clearMagnetDetection();
        bool magnetDetected;
    private:
        const static uint8_t sensor_pins_[NUM_MAGNETIC_SENSORS];
};

        // Be sure to modify NUM_MAGNETIC_SENSORS as well
        /*
        A0,
        A1,
            A2,
            A3,
            A4,
            A5,
            A6,
            A7,
            A8
        */

#endif