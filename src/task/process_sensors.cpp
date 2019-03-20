#include "process_sensors.h"


static Plotter plotter;

int16_t LEFTLIDARVAL;

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
    
    LEFTLIDARVAL = rangefinders.left.last_reading;

    //magnetics.logReadings();
    if (flameDetected) {
        DEBUG_PRINT("FLAME");
    }
    
    PLOTTER_SERIAL.print(rangefinders.left.last_reading);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(110*uint8_t(obj_l));
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(der_l);
    PLOTTER_SERIAL.print(",");
    PLOTTER_SERIAL.print(rangefinders.left.last_reading);
    PLOTTER_SERIAL.println(",");
    //print_object_data();
}
void print_object_data()
{
    for(int8_t i = 35; i > 0; i-=6)
    {
        Serial.print(objects[i-5]);
        Serial.print(" ");
        Serial.print(objects[i-4]);
        Serial.print(" ");
        Serial.print(objects[i-3]);
        Serial.print(" ");
        Serial.print(objects[i-2]);
        Serial.print(" ");
        Serial.print(objects[i-1]);
        Serial.print(" ");
        Serial.print(objects[i]);
        Serial.print("       ");
        Serial.print(confidence[i-5]);
        Serial.print(" ");
        Serial.print(confidence[i-4]);
        Serial.print(" ");
        Serial.print(confidence[i-3]);
        Serial.print(" ");
        Serial.print(confidence[i-2]);
        Serial.print(" ");
        Serial.print(confidence[i-1]);
        Serial.print(" ");
        Serial.println(confidence[i]);
    }
    Serial.println("space");
    Serial.println("space");
}