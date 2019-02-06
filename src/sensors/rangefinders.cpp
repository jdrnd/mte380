#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "rangefinders.h"

// Initializes the rangefinder and sets the timeout. Must be called before `run`
template <typename SensorType>
bool Rangefinder<SensorType>::init() {
    last_reading_ = 0;
    isRunning = false;
    sensor.setTimeout(500);
    sensor.init();
    return (sensor.last_status == 0);
}

// Enables continious reading on the sensor side so we can always read the most up-to-date value
template <typename SensorType>
void Rangefinder<SensorType>::run() {
    sensor.startContinuous(50);
    isRunning = true;
}

// Alias to the sensor API's `setAddress`
template <typename SensorType>
void Rangefinder<SensorType>::setAddress(uint8_t address) {
    sensor.setAddress(address);
}

// Returns the sensor's filtered reading in mm
// Using a simple exponential filter
template <typename SensorType>
uint16_t Rangefinder<SensorType>::read() {
    last_reading_ = RANGERFINDER_FILTER_ALPHA*sensor.readRangeContinuousMillimeters() + (1-RANGERFINDER_FILTER_ALPHA)*last_reading_;
    return last_reading_;
}

// Prints out the sensor's last reading (mm) to console
template <typename SensorType>
void Rangefinder<SensorType>::logLastData() {
    Serial.print(sensor.read());
}

// Initializes each range finder by placing them in standby (XSHUT LOW), then 
// enabling each in turn and setting a unique address
void Rangefinders::init() {
    Serial.println("Init");     
    
    pinMode(FRONT_LIDAR_PIN, OUTPUT);
    pinMode(BACK_LIDAR_PIN, OUTPUT);
    pinMode(LEFT_LIDAR_PIN, OUTPUT);
    pinMode(RIGHT_LIDAR_PIN, OUTPUT);
    pinMode(CLOSE_LIDAR_PIN, OUTPUT);

    // Place all LIDARS in standby
    digitalWrite(FRONT_LIDAR_PIN, LOW);
    digitalWrite(BACK_LIDAR_PIN, LOW);
    digitalWrite(LEFT_LIDAR_PIN, LOW);
    digitalWrite(RIGHT_LIDAR_PIN, LOW);
    digitalWrite(CLOSE_LIDAR_PIN, LOW);

    digitalWrite(FRONT_LIDAR_PIN, HIGH);
    if (!front.init())
    {
        Serial.println("Failed to detect and initialize front sensor!");
        while (1);
    }
    front.setAddress(FRONT_LIDAR_ADDRESS);

    digitalWrite(BACK_LIDAR_PIN, HIGH);
    if (!back.init())
    {
        Serial.println("Failed to detect and initialize back sensor!");
        while (1);
    }
    back.setAddress(BACK_LIDAR_ADDRESS);

    digitalWrite(LEFT_LIDAR_PIN, HIGH);
    if (!left.init())
    {
        Serial.println("Failed to detect and initialize left sensor!");
        while (1);
    }
    left.setAddress(LEFT_LIDAR_ADDRESS);

    digitalWrite(RIGHT_LIDAR_PIN, HIGH);
    if (!right.init())
    {
        Serial.println("Failed to detect and initialize right sensor!");
        while (1);
    }
    right.setAddress(RIGHT_LIDAR_ADDRESS);

    digitalWrite(CLOSE_LIDAR_PIN, HIGH);
    if (!close.init())
    {
        Serial.println("Failed to detect and initialize close sensor!");
        while (1);
    }
    close.setAddress(CLOSE_LIDAR_ADDRESS);
}

// Runs all LIDARS
void Rangefinders::run() {
    front.run();
    back.run();
    left.run();
    right.run();
    close.run();
}

// Logs readings from all LIDARS to console, comma seperated
void Rangefinders::logReadings() {
    Serial.print(front.read());
    Serial.print(',');
    Serial.print(back.read());
    Serial.print(',');
    Serial.print(left.read());
    Serial.print(',');
    Serial.print(right.read());
<<<<<<< HEAD
    Serial.print(',');
    Serial.print(close.read());
    Serial.print('\n');
=======
    Serial.print(",");
>>>>>>> Added Photosensor files
}
