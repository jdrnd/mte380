#include "process_sensors.h"


//static Plotter plotter;

int16_t LEFTLIDARVAL;

// this task is t_processSensors

void init_process_sensors() {
    #ifndef RUN_LOGGING
        t_processSensors.disable();
        return;
    #endif

    t_processSensors.setCallback(&process_sensors);
}
void process_sensors() {
    LEFTLIDARVAL = rangefinders.left.last_reading;
    #ifdef RUN_IMU
    Accel accel = imu->getAccel();
    Orientation ypr = imu->getYPR();
    #endif

    Magnetics::logReadings();
    if (flameDetected) {
        //DEBUG_PRINT("FLAME");
    }

    //print_object_data();
}