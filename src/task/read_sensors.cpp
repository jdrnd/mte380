#include "read_sensors.h"

// current task = t_readSensors

void init_sensors() {
    DEBUG_PRINT("Init Read Sensors Task");

    // Start I2C bus for IMU and LIDARs
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    imu->init();
    imu->run();
    
    rangefinders.init();
    rangefinders.run();

    read_sensors();
    // The next runs of this task will use the read_sensors callback function
    t_readSensors.setCallback(&read_sensors);
}

void read_sensors() {
    candleSensor.read();
    imu->readData();
    rangefinders.readAll();
}