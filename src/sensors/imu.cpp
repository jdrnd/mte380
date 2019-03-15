#include "imu.h"

/*
Some of the code in this file was taken from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
*/

// The "static" IMU object that our ISR will access
IMU* IMU_Wrapper::primary = new IMU;

// Sets up and initializes the IMU
// takes about 500 ms to run
bool IMU::init() {
    yaw_offset_ = 0;
    data_ready_ = false;

    // Add initial measurement to IMU buffers for filtering
    measurements_x_.push(0);
    measurements_y_.push(0);
    measurements_z_.push(0);
    yaw_.push(0);
    pitch_.push(0);
    roll_.push(0);

    accel = Accel{};
    orientation = Orientation{};

    mpu_ = MPU6050();

    uint8_t devStatus = mpu_.dmpInitialize();

    // Offsets determined experimentally
    mpu_.setXAccelOffset(-2818);
    mpu_.setYAccelOffset(1090);
    mpu_.setZAccelOffset(912);
    mpu_.setXGyroOffset(3);
    mpu_.setYGyroOffset(77);
    mpu_.setZGyroOffset(-11);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu_.setDMPEnabled(true);
        
        // get expected DMP packet size for later comparison
        imu_packetsize_ = mpu_.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DEBUG_PRINT(F("DMP Initialization failed (code "));
        DEBUG_PRINT(devStatus);
        DEBUG_PRINT(F(")"));
        while(true){};
    }
    return true;
}

void IMU::run() {
    // Start responding to the IMU's interrupts
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), IMU_Wrapper::onDataReady, RISING);
}

// Interrupt that tells us the IMU has new data
void IMU_Wrapper::onDataReady() {
    //DEBUG_PRINT("interrupted");
    imu->data_ready_ = true;
}

// ISR that is run every time the IMU has a new reading
void IMU::readData() {
    //mpu_.resetFIFO();
    //data_ready_ = false;

    DEBUG_PRINT("waiting for IMU")
    // wait for correct available data length, should be a VERY short wait
    uint16_t fifoCount = mpu_.getFIFOCount();
    while (fifoCount < imu_packetsize_) {
        fifoCount = mpu_.getFIFOCount();
        DEBUG_PRINT("nodata")
    }

    DEBUG_PRINT("passedwait");
    DEBUG_PRINT("imu has bytes");
    // read a packet from FIFO
    uint8_t imu_buffer[64];
    mpu_.getFIFOBytes(imu_buffer, imu_packetsize_);

    DEBUG_PRINT("gotbytes bytes");
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= imu_packetsize_;

    Quaternion q;   // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector 

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu_.dmpGetQuaternion(&q, imu_buffer);
    mpu_.dmpGetAccel(&aa, imu_buffer);
    mpu_.dmpGetGravity(&gravity, &q);
    mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu_.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu_.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    measurements_x_.push(IMU_FILTER_ALPHA * aaWorld.x + (1-IMU_FILTER_ALPHA)*measurements_x_.last());
    accel.x = ((1.0*measurements_x_.last() / 16384.0) * 9.81);
    measurements_y_.push(IMU_FILTER_ALPHA * aaWorld.y + (1-IMU_FILTER_ALPHA)*measurements_y_.last());
    accel.y = ((1.0*measurements_y_.last() / 16384.0) * 9.81);
    measurements_z_.push(IMU_FILTER_ALPHA * aaWorld.z + (1-IMU_FILTER_ALPHA)*measurements_z_.last());
    accel.z = ((1.0*measurements_z_.last() / 16384.0) * 9.81);

    yaw_.push((IMU_FILTER_ALPHA*(ypr[0] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*yaw_.last()) - yaw_offset_);
    orientation.yaw = yaw_.last();
    pitch_.push(IMU_FILTER_ALPHA*(ypr[1] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*pitch_.last());
    orientation.pitch = pitch_.last();
    roll_.push(IMU_FILTER_ALPHA*(ypr[2] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*roll_.last());
    orientation.roll = roll_.last();

    DEBUG_PRINT("doneimu");

    mpu_.resetFIFO();
}

void IMU::zero_yaw() {
    yaw_offset_ = orientation.yaw;
}

// Returns x,y,z acceleration as a struct
Accel IMU::getAccel() {
    return Accel{
        ((double)measurements_x_.last() / 16384) * 9.81,
        ((double)measurements_y_.last() / 16384) * 9.81,
        ((double)measurements_z_.last() / 16384) * 9.81
    };
}

// Return Yaw, Pitch, Roll of sensor
Orientation IMU::getYPR() {
    return Orientation{
        yaw_.last(),
        pitch_.last(),
        roll_.last()
    };
}