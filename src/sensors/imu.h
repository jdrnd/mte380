#ifndef IMU_H
#define IMU_H

/* 
    Wiring:

    MPU 6050 || Arudino
    5V -- 5V supply
    GND -- GND
    SDA -- SDA
    SCL -- SCL
    INT -- Interupt-capable digital IO pin, defined by IMU_INTERRUPT_PIN
*/

#include <MPU6050_6Axis_MotionApps20.h>

// From the circular buffer library, internally optimizes so it
// is safer to access the buffer from ISRs
#define CIRCULAR_BUFFER_INT_SAFE
#include <CircularBuffer.h>

#include "common.h"

#define IMU_INTERRUPT_PIN 3

// Number of values of history to keep
#define IMU_BUFFER_LEN 10

// Simple exponential filter to reduce noise, see https://en.wikipedia.org/wiki/Exponential_smoothing
// 0 < ALPHA <= 1, lower alpha means more smoothing
#define IMU_FILTER_ALPHA 0.8

// This way we can refer to `imu` in our application code
#define imu IMU_Wrapper::primary

// Represents filtered, normalized accerleration values in m/s^2
struct Accel {
    double x;
    double y;
    double z;
};

// Normalized, filtered orientation in degrees
struct Orientation {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
};

// We use a seperate wrapper class with a static member
// so that we can access the IMU from within an interrupt
// service routine
class IMU_Wrapper;

// Our MPU 6050 IMU
class IMU {
    friend class IMU_Wrapper;

    public:
        bool init();
        void run();

        void readData();

        Accel accel;
        Orientation orientation;

        Accel getAccel();
        Orientation getYPR();

        void zero_yaw();
    private:
        CircularBuffer<int16_t, IMU_BUFFER_LEN> measurements_x_;
        CircularBuffer<int16_t, IMU_BUFFER_LEN> measurements_y_;
        CircularBuffer<int16_t, IMU_BUFFER_LEN> measurements_z_;

        CircularBuffer<int16_t, IMU_BUFFER_LEN> yaw_;
        CircularBuffer<int16_t, IMU_BUFFER_LEN> pitch_;
        CircularBuffer<int16_t, IMU_BUFFER_LEN> roll_;

        uint16_t yaw_offset_;

        // class default I2C address is 0x68
        MPU6050 mpu_;
        uint16_t imu_packetsize_;

        volatile bool data_ready_;
};

class IMU_Wrapper {
    public:
        static IMU* primary;
        static void onDataReady();
};

#endif