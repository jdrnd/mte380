#ifndef CONFIG_H
#define CONFIG_H

/*
    Common header files for all tasks, do not remove
*/
#include "etl_profile.h"
#include "ecl_user.h"
#include <ArduinoSTL.h>


/*
    Configuration values for each run
*/

// State_t::OBJECT_SEARCH, State_t::FIND_MAGNET, State_t::CANDLE_SEARCH
#define STARTING_STATE State_t::OBJECT_SEARCH
#define STARTING_X_POS 5
#define STARTING_Y_POS 3

#define STARTING_ORIENTATION 2// along y-axis, towards center


/*
    Functionality flags
*/

//#define NO_MOTOR_RAMP 
// Start with the map state known
#define USE_KNOWN_MAP
#define RUN_LOGGING
#define RUN_LIDARS
//#define RUN_IMU
//#define STOP_ON_MAGNET
//#define LOG_MOTOR_CONTROL
//#define LOG_OBJECT_DETECTION


/*
    Macros for debugging
*/

#define PLOTTER_SERIAL Serial3 // This must be set in the platformio file as well
#define DEBUG_PRINT(x) Serial.print(millis()); Serial.print(" "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x);

#endif // CONFIG_H