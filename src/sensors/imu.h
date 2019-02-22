#ifndef IMU_H
#define IMU_H

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 3.
 * ========================================================================= */

#include <MPU6050_6Axis_MotionApps20.h>

#include <CircularBuffer.h>

#define OUTPUT_READABLE_WORLDACCEL
#define CIRCULAR_BUFFER_INT_SAFE

#define IMU_INTERRUPT_PIN 3

#define imu IMU_Wrapper::primary

struct Accel {
    double x;
    double y;
    double z;
};

class IMU_Wrapper;

class IMU {
    friend class IMU_Wrapper;

    public:
        bool init();
        void run();

        void onInterupt();
    private:
        
        CircularBuffer<int16_t, 100> measurements;

        // class default I2C address is 0x68
        MPU6050 mpu_;
        uint16_t imu_packetsize_;
        uint8_t imu_buffer_[64];

        Quaternion q_;   // [w, x, y, z]         quaternion container
        VectorInt16 aa_;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal_;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld_;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity_;    // [x, y, z]            gravity vector
        float euler_[3];         // [psi, theta, phi]    Euler angle container
        float ypr_[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector 
};

class IMU_Wrapper {
    public:
        static IMU* primary;
        static void onPrimaryInterrupt();
};

#endif