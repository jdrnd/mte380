#ifndef RANGEFINDERS_H
#define RANGEFINDERS_H

#include <VL53L1X.h>

#define FRONT_LIDAR_PIN 5
#define BACK_LIDAR_PIN 6
#define LEFT_LIDAR_PIN 7
#define RIGHT_LIDAR_PIN 8

#define FRONT_LIDAR_ADDRESS 0x30
#define BACK_LIDAR_ADDRESS 0x31
#define LEFT_LIDAR_ADDRESS 0x32
#define RIGHT_LIDAR_ADDRESS 0x33

/*
Requires that Wire.begin() and Wire.setClock() are called first.

Wiring:
- Each sensor's VIN and GND should be connected to power and ground respectively.
- Each sensor's SCL and SDA should be connected to the two *shared* I2C lines, and the arduino's pins
- Each sensor's XSHUT pin should be connected to a unique digital pin on the arduino, according to the numbers above
*/


class Rangefinder {
    public:
        bool init();
        void run();

        uint16_t read();
        void logLastData();
        void setAddress(uint8_t);

        bool isRunning;
    private:
        VL53L1X sensor;
};

class Rangefinders
{
    public:
        void init();
        void run();

        void logReadings();

        Rangefinder front;
        Rangefinder back;
        Rangefinder left;
        Rangefinder right;
        Rangefinder shortrange;
};
#endif