#ifndef CONFIG_H
#define CONFIG_H

/*
    Runtime config file options
*/
#include "etl_profile.h"
#include "ecl_user.h"
#include <ArduinoSTL.h>

// Start with the map state known
//#define USE_KNOWN_MAP

#define RUN_LOGGING

//#define RUN_LIDARS
//#define RUN_IMU

//#define LOG_MOTOR_CONTROL

// This must be set in the platformio file as well
#define PLOTTER_SERIAL Serial3

#define DEBUG_PRINT(x) Serial.print(millis()); Serial.print(" "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x);
#endif