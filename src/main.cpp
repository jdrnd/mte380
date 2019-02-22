#include "sensors/imu.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("Setup beginning");

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    imu->init();
    imu->run();
}

void loop() {
    delay(100);
    
    Accel accel = imu->getAccel();
    Serial.print("Accel: ");
    Serial.print(accel.x);
    Serial.print(",");
    Serial.print(accel.y);
    Serial.print(",");
    Serial.println(accel.x);

    Orientation ypr = imu->getYPR();
    Serial.print("YPR: ");
    Serial.print(ypr.yaw);
    Serial.print(",");
    Serial.print(ypr.pitch);
    Serial.print(",");
    Serial.println(ypr.roll);

}