#include "sensors/imu.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("Setup beginning");

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    imu->init();
}

void loop() {
}