#include "process_sensors.h"


static Plotter plotter;

// this task is t_processSensors

void init_process_sensors() {
    #ifndef RUN_LOGGING
        t_processSensors.disable();
        return;
    #endif

    /*
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
    plotter.AddTimeGraph("Colors", 100, "Red", colorsensor.r, "Blue", colorsensor.b, "Green", colorsensor.g);
    */

    t_processSensors.setCallback(&process_sensors);
}
void process_sensors() {
    DEBUG_PRINT("Process Sensors")

    Accel accel = imu->getAccel();
    Orientation ypr = imu->getYPR();

    //magnetics.logReadings();
    if (flameDetected) {
        DEBUG_PRINT("FLAME");
    }
    
    //PLOTTER_SERIAL.print(rangefinders.shortrange.last_reading);
    //PLOTTER_SERIAL.print(",");
    //PLOTTER_SERIAL.println((int)colorsensor.curr_terrain);
    
    // PLOTTER_SERIAL.print(rangefinders.front.last_reading);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.print(rangefinders.back.last_reading);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.print(rangefinders.left.last_reading);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.print(rangefinders.right.last_reading);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.println(rangefinders.shortrange.last_reading);


    // PLOTTER_SERIAL.print(accel.x);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.print(imu->orientation.pitch);
    // PLOTTER_SERIAL.print(",");
    // PLOTTER_SERIAL.println(imu->orientation.roll);
    
    // Implicitly accesses varaibles set in the init step
    //plotter.Plot();
}