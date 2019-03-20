#include "process_sensors.h"


static Plotter plotter;

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
    DEBUG_PRINT("Process Sensors")
    
    LEFTLIDARVAL = rangefinders.left.last_reading;

    //magnetics.logReadings();
    if (flameDetected) {
        DEBUG_PRINT("FLAME");
    }
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