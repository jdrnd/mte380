#include <Arduino.h>
#include <Wire.h>

#include "photosensor.h"

// Returns the sensor's last reading
uint16_t Photosensor::read() {
    return analogRead(PHOTOSENSOR_PIN);
}

// Logs readings from photosensor to console
void Photosensor::logLastReading() {
    Serial.println(read());
}
