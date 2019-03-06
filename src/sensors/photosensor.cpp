#include <Arduino.h>
#include <Wire.h>

#include "photosensor.h"

// Returns the sensor's last reading
uint16_t Photosensor::read() {
    last_reading = analogRead(PHOTOSENSOR_PIN);
    return last_reading;
}

// Logs readings from photosensor to console
void Photosensor::logLastReading() {
    DEBUG_PRINT(read());
}
