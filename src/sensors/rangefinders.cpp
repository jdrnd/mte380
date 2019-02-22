#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "rangefinders.h"

// Initializes the rangefinder and sets the timeout. Must be called before `run`
bool Rangefinder::init() {
    isRunning = false;
    sensor.setTimeout(500);
    return sensor.init();
}

// Enables continious reading on the sensor side so we can always read the most up-to-date value
void Rangefinder::run() {
    sensor.startContinuous(50);
    isRunning = true;
}

// Alias to the sensor API's `setAddress`
void Rangefinder::setAddress(uint8_t address) {
    sensor.setAddress(address);
}

// Returns the sensor's last reading
uint16_t Rangefinder::read() {
    sensor.read();
    return sensor.ranging_data.range_mm;
}

// Prints out the sensor's last reading (mm) to console
void Rangefinder::logLastData() {
    //Serial.println(sensor.getAddress());
    sensor.read();
    
    Serial.print(sensor.ranging_data.range_mm);

    /*
    Serial.print("\tstatus: ");
    Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
    Serial.print("\tpeak signal: ");
    Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
    Serial.print("\tambient: ");
    Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
    */
}

// Initializes each range finder by placing them in standby (XSHUT LOW), then 
// enabling each in turn and setting a unique address
void Rangefinders::init() {
    Serial.println("Init");
    
    pinMode(FRONT_LIDAR_PIN, OUTPUT);
    pinMode(BACK_LIDAR_PIN, OUTPUT);
    pinMode(LEFT_LIDAR_PIN, OUTPUT);
    pinMode(RIGHT_LIDAR_PIN, OUTPUT);

    // Place all LIDARS in standby
    digitalWrite(FRONT_LIDAR_PIN, LOW);
    digitalWrite(BACK_LIDAR_PIN, LOW);
    digitalWrite(LEFT_LIDAR_PIN, LOW);
    digitalWrite(RIGHT_LIDAR_PIN, LOW);

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
}

// Runs all LIDARS
void Rangefinders::run() {
    front.run();
    back.run();
    left.run();
    right.run();
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
    Serial.print('\n');
}
