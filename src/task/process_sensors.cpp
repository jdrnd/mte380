#include "process_sensors.h"

static Accel accel;
static Orientation ypr;

static Plotter plotter;

// this task is t_processSensors

void init_process_sensors() {
    #ifndef RUN_LOGGING
        t_processSensors.disable();
        return;
    #endif

    plotter.Begin();
    plotter.AddTimeGraph("Accelerometer", 200, "X", accel.x, "Y", accel.y, "Z", accel.z);
    plotter.AddTimeGraph("Motors", 200, "Left", motors.left->speed, "Right", motors.right->speed);
    plotter.AddTimeGraph("Lidars", 200, 
            "Front", rangefinders.front.last_reading, 
            "Back", rangefinders.back.last_reading,
            "Left", rangefinders.left.last_reading,
            "Right", rangefinders.right.last_reading,
            "Shortrange", rangefinders.short.last_reading
    );
    plotter.AddTimeGraph("", 200, "Light Sensor", candleSensor.last_reading);

    t_processSensors.setCallback(&process_sensors);
}
void process_sensors() {
    Accel currentAccel = imu->getAccel();
    accel = {
        currentAccel.x,
        currentAccel.y,
        currentAccel.z
    };
    
    // Implicitly accesses varaibles set in the init step
    plotter.Plot();
}