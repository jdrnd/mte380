#include "imu.h"


IMU* IMU_Wrapper::primary = new IMU;

bool IMU::init() {
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
        

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), IMU_Wrapper::onPrimaryInterrupt, RISING);
        
        delay(3000);

        Serial.println("noerror");
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

void IMU_Wrapper::onPrimaryInterrupt() {
    //Serial.println("interrupted");
    imu->onInterupt();
}

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
        mpu_.getFIFOBytes(imu_buffer_, imu_packetsize_);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= imu_packetsize_;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu_.dmpGetQuaternion(&q, imu_buffer_);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu_.dmpGetQuaternion(&q, imu_buffer_);
            mpu_.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu_.dmpGetQuaternion(&q, imu_buffer_);
            mpu_.dmpGetGravity(&gravity, &q);
            mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_RAW_GYRO
            uint32_t data[3];
            mpu_.dmpGetGyro(data, imu_buffer_);
            Serial.print(data[0]);
            Serial.print(",");
            Serial.print(data[1]);
            Serial.print(",");
            Serial.println(data[2]);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu_.dmpGetQuaternion(&q, imu_buffer_);
            mpu_.dmpGetAccel(&aa, imu_buffer_);
            mpu_.dmpGetGravity(&gravity, &q);
            mpu_.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print(aaReal.x);
            Serial.print(",");
            Serial.print(aaReal.y);
            Serial.print(",");
            Serial.println(aaReal.z);
        #endif
        noInterrupts();

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu_.dmpGetQuaternion(&q_, imu_buffer_);
            mpu_.dmpGetAccel(&aa_, imu_buffer_);
            mpu_.dmpGetGravity(&gravity_, &q_);
            mpu_.dmpGetLinearAccel(&aaReal_, &aa_, &gravity_);
            mpu_.dmpGetLinearAccelInWorld(&aaWorld_, &aaReal_, &q_);
            Serial.print(aaWorld_.x);
            Serial.print(",");
            Serial.print(aaWorld_.y);
            Serial.print(",");
            Serial.println(aaWorld_.z);
        #endif
    }
}