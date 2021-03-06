#ifndef PHOTOSENSOR_H
#define PHOTOSENSOR_H

#define PHOTOSENSOR_PIN A0

#include "common.h"
/*
Requires that Wire.begin() and Wire.setClock() are called first.

Wiring:
- Sensor's VIN and GND should be connected to power and ground respectively.
- Sensor's signal pin is to be connected to digital pin on Arduino Meda as defined above
*/

class Photosensor {
    public:
        uint16_t read();
        void logLastReading();
        uint16_t last_reading;
        //Possibly add running average or circular buffer
};

#endif