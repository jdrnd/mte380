#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "common.h"


#define GYRO_SCALING_FACTOR 130.1777777
class Gyro {
    public:
        void init();
        void read();

        float roll = 0, pitch = 0, yaw = 0;
    private:
        const int MPU_addr=0x68;  // I2C address of the MPU-6050
        
        //WARNING TIME OVERFLOWS AT SOME POINT!!!!!! (ATLEAST 70 MINUTES THOUGH I THINK)
        float time = 0, lastTime = 0, dt = 0; 
        float GyX_Mean_Sum = 0, GyY_Mean_Sum = 0, GyZ_Mean_Sum = 0;
        float ax = 0, ay = 0, az = 0;

        int32_t x[15] = {0}, x_max = -100000, x_min = 100000;
        int32_t y[15] = {0}, y_max = -100000, y_min = 100000;
        int32_t z[15] = {0}, z_max = -100000, z_min = 100000;
        //WARNING "i" MAY OVERFLOW AT SOME POINT
        int32_t x_offset = 0, y_offset = 0, z_offset = 0, i = 0;
        int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

        MPU6050 mpu;
};

#endif