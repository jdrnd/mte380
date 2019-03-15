#ifndef RANGEFINDERS_H
#define RANGEFINDERS_H

#include <VL53L1X.h>
#include <VL6180X.h>
#include <CircularBuffer.h>

#include "common.h"

#define FRONT_LIDAR_PIN 45
#define BACK_LIDAR_PIN 44
#define LEFT_LIDAR_PIN 43
#define RIGHT_LIDAR_PIN 42
#define CLOSE_LIDAR_PIN 46

#define FRONT_LIDAR_ADDRESS 0x30
#define BACK_LIDAR_ADDRESS 0x31
#define LEFT_LIDAR_ADDRESS 0x32
#define RIGHT_LIDAR_ADDRESS 0x33
#define CLOSE_LIDAR_ADDRESS 0x34

#define RANGERFINDER_FILTER_ALPHA 0.8

#define BUFF_SIZE 100

//offsets to the lidars from the IMU, in mm
#define RIGHT_LIDAR_OFFSET 50
#define LEFT_LIDAR_OFFSET 50
#define FRONT_LIDAR_OFFSET 100
#define BACK_LIDAR_OFFSET 100

/*
Requires that Wire.begin() and Wire.setClock() are called first.

Wiring:
- Each sensor's VIN and GND should be connected to power and ground respectively.
- Each sensor's SCL and SDA should be connected to the two *shared* I2C lines, and the arduino's pins
- Each sensor's XSHUT pin should be connected to a unique digital pin on the arduino, according to the numbers above
*/

// We use templates here so that we can use both the 
// provided short and longrange sensor libraries without
// Re-writing a bunch of code, see the Rangefinders class
template<typename SensorType>
class Rangefinder {
    public:
        bool init();
        void run();

        uint16_t read();
        void logLastData();
        void setAddress(uint8_t);

        uint16_t last_reading;

        bool isRunning;

        //Moved to public, having difficulties with a pointer to a variable class like circular buffer
        CircularBuffer<uint16_t, BUFF_SIZE> readings_;

    private:
        SensorType sensor;
};

class Rangefinders
{
    public:
        void init();
        void run();

        void readAll();
        void logReadings();

        Rangefinder<VL53L1X> front;
        Rangefinder<VL53L1X> back;
        Rangefinder<VL53L1X> left;
        Rangefinder<VL53L1X> right;
        Rangefinder<VL6180X> shortrange;
};
#endif