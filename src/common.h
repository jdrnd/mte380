#ifndef CONFIG_H
#define CONFIG_H

/*
    Runtime config file options
*/

#define RUN_LOGGING

#define RUN_LIDARS
#define RUN_IMU

#define LOG_MOTOR_CONTROL

extern float theta_x;
extern float theta_y;
extern float theta_z;

// This must be set in the platformio file as well
#define PLOTTER_SERIAL Serial3

#define DEBUG_PRINT(x) Serial.print(millis()); Serial.print(" "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x);
#endif