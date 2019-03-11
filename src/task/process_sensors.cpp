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
    plotter.AddTimeGraph("Accelerometer", 100, "X", accel.x, "Y", accel.y, "Z", accel.z);
    plotter.AddTimeGraph("Gyro", 100, "Yaw", ypr.yaw, "Pitch", ypr.pitch, "Roll", ypr.roll);
    plotter.AddTimeGraph("Motors", 100, "Left", motors.left->speed, "Right", motors.right->speed, "Left setpoint", motors.left->speed_setpoint, "Right setpoint", motors.right->speed_setpoint);
    plotter.AddTimeGraph("Lidars", 100, 
            "Front", rangefinders.front.last_reading, 
            "Back", rangefinders.back.last_reading,
            "Left", rangefinders.left.last_reading,
            "Right", rangefinders.right.last_reading,
            "Shortrange", rangefinders.shortrange.last_reading
    );
    plotter.AddTimeGraph("", 100, "Light Sensor", candleSensor.last_reading);
    plotter.AddTimeGraph("Encoder distances", 100, "Left", motors.left->distance, "Right", motors.right->distance);

    t_processSensors.setCallback(&process_sensors);
}
void process_sensors() {
    Accel currentAccel = imu->getAccel();
    accel = {
        currentAccel.x,
        currentAccel.y,
        currentAccel.z
    };
    Orientation orient = imu->getYPR();
    ypr = {
        orient.yaw,
        orient.pitch,
        orient.roll
    };
    
    // Implicitly accesses varaibles set in the init step
    plotter.Plot();
}