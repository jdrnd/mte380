#ifdef teensy

#include <Arduino.h>

#include <Wire.h>

#include "sensors/rangefinders.h"

Rangefinders lidars;

void setup() {
    Serial.begin(115200);
    while(!Serial) {} // REQURIED for teensy!!
    Serial.println("Setup beginning");

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    lidars.init();
    lidars.run();
    Serial.println("Setup complete");
}

void loop() {
    lidars.logReadings();
    delay(200);
}

#endif
