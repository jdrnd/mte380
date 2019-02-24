#include <Arduino.h>

/*
    TODO: Mount sensors on robot and determine pinout+tolerancing
    Usable pins: A0-A14 on mega, harder but possible on teensy
    Overall usage: Magnetics::detectMagnet() returns whether or not a magnet
    was detected on any sensor
*/

#define HALL_EFFECT_TOL 3
#define HALL_EFFECT_MEAN 510 // experimentally measured

// Modify here to change number of sensors
// And in magnetics class to add sensor pins
#define NUM_MAGNETIC_SENSORS 1
extern uint8_t sensor_pins[NUM_MAGNETIC_SENSORS];

class Magnetics;

class Magnetic {
    friend class Magnetics;

    public:
        bool detectMagnet();
    private:
        uint8_t sensor_pin_;
};

class Magnetics {
    public:
        Magnetics();
        static bool detectMagnet();
    private:
        // Be sure to modify NUM_MAGNETIC_SENSORS as well
        static constexpr uint8_t sensor_pins_[NUM_MAGNETIC_SENSORS] = {
            A0
        };
        static Magnetic sensors_[NUM_MAGNETIC_SENSORS];
};