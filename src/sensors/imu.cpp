#include "imu.h"

// The "static" IMU object that our ISR will access
IMU* IMU_Wrapper::primary = new IMU;

// Sets up and initializes the IMU
bool IMU::init() {
    // Add initial measurement to IMU buffers for filtering
    measurements_x_.push(0);
    measurements_y_.push(0);
    measurements_z_.push(0);
    yaw_.push(0);
    pitch_.push(0);
    roll_.push(0);

    mpu_ = MPU6050();

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    delay(1000);
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
        delay(1000);
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_.setDMPEnabled(true);
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        
        // get expected DMP packet size for later comparison
        imu_packetsize_ = mpu_.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    return true;
}

void IMU::run() {
    // Start responding to the IMU's interrupts
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), IMU_Wrapper::onPrimaryInterrupt, RISING);
}


// Wrapper so we can access the IMU class instance via a "static" function
void IMU_Wrapper::onPrimaryInterrupt() {
    //Serial.println("interrupted");
    imu->onInterupt();
}

// ISR that is run every time the IMU has a new reading
void IMU::onInterupt() {
    // This is a bit sketchy, but what we do here is within our ISR we 
    // re-enable interrupts so that reading from the I2C bus will work
    // This way we can process IMU values as soon as they are available
    // Without affecting the rest of our program
    interrupts();

    uint8_t mpu_IntStatus = mpu_.getIntStatus();
    uint16_t fifoCount = mpu_.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu_IntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu_.resetFIFO();
        fifoCount = mpu_.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu_IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < imu_packetsize_) fifoCount = mpu_.getFIFOCount();

        // read a packet from FIFO
        uint8_t imu_buffer[64];
        mpu_.getFIFOBytes(imu_buffer, imu_packetsize_);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= imu_packetsize_;
        noInterrupts();

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
        measurements_y_.push(IMU_FILTER_ALPHA * aaWorld.y + (1-IMU_FILTER_ALPHA)*measurements_y_.last());
        measurements_z_.push(IMU_FILTER_ALPHA * aaWorld.z + (1-IMU_FILTER_ALPHA)*measurements_z_.last());

        yaw_.push(IMU_FILTER_ALPHA*(ypr[0] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*yaw_.last());
        pitch_.push(IMU_FILTER_ALPHA*(ypr[1] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*pitch_.last());
        roll_.push(IMU_FILTER_ALPHA*(ypr[2] * 180/M_PI) + (1-IMU_FILTER_ALPHA)*roll_.last());

        #ifdef DEBUG_YPR
            Serial.print(yaw_.last());
            Serial.print(",");
            Serial.print(pitch_.last());
            Serial.print(",");
            Serial.println(roll_.last());
        #endif

    }
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