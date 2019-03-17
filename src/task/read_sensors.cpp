#include <Wire.h>
//#include <I2Cdev.h>

#include "read_sensors.h"

// current task = t_readSensors

void init_sensors() {
    DEBUG_PRINT("Init Read Sensors Task");

    // Start I2C bus for IMU and LIDARs
    Wire.begin();
    Wire.setClock(400000); // use 100 kHz I2C , 400K may cause crashes??
    //Fastwire::setup(400, true);

    #ifdef RUN_IMU
    imu->init();
    imu->run();
    #endif
    
    #ifdef RUN_LIDARS
    rangefinders.init();
    rangefinders.run();
    #endif

    IR::init();
    //gyro.init();

    colorsensor.initialize();

    read_sensors();
    // The next runs of this task will use the read_sensors callback function
    t_readSensors.setCallback(&read_sensors);
}

void read_sensors() {
    DEBUG_PRINT("Read Sensors");
    candleSensor.read();

    #ifdef RUN_IMU
    imu->readData();
    #endif

    #ifdef RUN_LIDARS
    rangefinders.readAll();
    #endif

    IR::read();
    magnetics.detectMagnet();

    detectFlame();

    #ifdef RUN_GYRO2
    gyro.read();
    #endif
    
    colorsensor.read_terrain(rangefinders.shortrange.last_reading);

    motors.left->readDistance();
    motors.right->readDistance();

    DEBUG_PRINT("End Read Sensors");
}