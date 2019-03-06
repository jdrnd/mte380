#include <Arduino.h>
#include <Wire.h>

#include "sensors/rangefinders.h"
#include "sensors/photosensor.h"

Rangefinders rangefinders;
Photosensor candleSensor;

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
    candleSensor.logLastReading();
     Serial.print("\n");
    //localization(returns x,y,theta)
    delay(200);
}