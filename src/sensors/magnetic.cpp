#include <Arduino.h>

#include "magnetic.h"


namespace Magnetics {
    CircularBuffer<uint16_t, 10> sensor_values_[NUM_MAGNETIC_SENSORS];
    bool magnetDetected = false;

    const uint8_t sensor_pins_[NUM_MAGNETIC_SENSORS] = {
        A0,
        A1,
        A2,
        A3,
        A4,
        A5,
        A6,
        A7,
        A8
    };

void logReadings() {
    // for (uint8_t i=0; i<sizeof(MAGNETIC_PINS)/sizeof(uint8_t); i++) {
    //     PLOTTER_SERIAL.print(analogRead(Magnetics::sensor_pins_[i]));
    //     PLOTTER_SERIAL.print(",");
    // }
    // PLOTTER_SERIAL.println("0");
}

void detectMagnet() {
    for (uint8_t i=0; i<NUM_MAGNETIC_SENSORS; i++){
        int16_t sensor_avg = 0;

        for (uint8_t j = 0; j < sensor_values_[i].size(); j++) {
			sensor_avg += sensor_values_[i][j];
		}
        sensor_avg /= sensor_values_[i].size();

        int16_t sensor_reading = analogRead(Magnetics::sensor_pins_[i]);
        sensor_values_[i].push((uint16_t)sensor_reading);

        
        if (sensor_values_[i].size() > 6) {
            if (abs(sensor_reading - sensor_avg) > HALL_EFFECT_TOL && abs(sensor_values_[i][sensor_values_[i].size() -2] - sensor_avg) > HALL_EFFECT_TOL2) {
                magnetDetected = true;
                return;
            }
        }
    }
}

    void clearMagnetDetection() {
        magnetDetected = false;
    }
};
