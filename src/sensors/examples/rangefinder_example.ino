#include <Arduino.h>
#include <Wire.h>

#include "sensors/rangefinders.h"

Rangefinders rangefinders;

void setup() {
    Serial.begin(115200);
    Serial.println("Setup beginning");

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    rangefinders.init();
    rangefinders.run();
    Serial.println("Setup complete");
}

void loop() {
    rangefinders.logReadings();
    delay(200);
}