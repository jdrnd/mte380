#include "gyro.h"

void Gyro::init(){
    /*
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    */
    mpu.initialize();
    mpu.setXAccelOffset(-3705);
    mpu.setYAccelOffset(445);
    mpu.setZAccelOffset(1662);
    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(9);
    mpu.setZGyroOffset(12);
}

void Gyro::read(){
    DEBUG_PRINT("begin IMU read"); 
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    /*
    //READS ACCEL AND GYRO RAW DATA FROM IMU
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    */
    
    /*
    //STORES CIRCULAR BUFFER OF LAST 30 GYRO DATA POINTS (USED TO GET OFFSET VALUES LATER)
    x[i%15] = GyX;
    y[i%15] = GyY;
    z[i%15] = GyZ;

    //DETERMINE IF GYRO HAS BEEN STATIONARY FOR LAST 30 DATA POINTS, IF SO WE CAN REZERO TO GET NEW OFFSET VALUES AND PREVENT DRIFT
    //DETERMINE MAX AND MIN VALUES FROM LAST 30 DATA POINTS FOR EACH AXIS
    for(int i = 0; i < 15; i++){
        if(x[i] > x_max)
            x_max = x[i];
        else if(x[i] < x_min)
            x_min = x[i];
        else if(y[i] > y_max)
            y_max = y[i];
        else if(y[i] < y_min)
            y_min = y[i];
        if(z[i] > z_max)
            z_max = z[i];
        else if(z[i] < z_min)
            z_min = z[i];
    }

    //IF RANGE OF LAST 30 VALUES IS WITHIN A 100 UNIT WINDOW, IMU HAS BEEN STATIONARY SUFFICIENTLY LONG ENOUGH TO ALLOW REZEROING
    if((x_max-x_min) < 100){
        GyX_Mean_Sum = 0;
        for(int i = 0; i < 15; i++){
            GyX_Mean_Sum = GyX_Mean_Sum + x[i];
        }
        x_offset = GyX_Mean_Sum / 15;
    }
    if((y_max-y_min) < 100){
        GyY_Mean_Sum = 0;
        for(int i = 0; i < 15; i++)
            GyY_Mean_Sum = GyY_Mean_Sum + y[i];
        
        y_offset = GyY_Mean_Sum / 15;
    }
    if((z_max-z_min) < 100){
        GyZ_Mean_Sum = 0;
        for(int i = 0; i < 15; i++)
            GyZ_Mean_Sum = GyZ_Mean_Sum + z[i];
            
        z_offset = GyZ_Mean_Sum / 15;
    }

    //RESET MAX AND MIN VARIABLES FOR DATA BUFFER
    x_max = -100000;
    y_max = -100000;
    z_max = -100000;
    x_min = 100000;
    y_min = 100000;
    z_min = 100000;
    */
    //16384 = 1g, CONVERT ACCELERATION TO Gs
    ax = AcX / 16384.;
    ay = AcY / 16384.;
    az = AcZ / 16384.;

    //DETERMINE TIME STEP FOR INTEGRATING ANGULAR VELOCITY
    lastTime = time;
    time = micros();
    dt = time - lastTime;

    //COUNTER TO INCREMENT CIRCULAR BUFFER
    i++;

    //UPDATE YAW PITCH, AND ROLL
    roll = roll + (GyX* dt / 1000000.);//130.17777777 IS SCALING CONST TO GET RAD ANGULAR DATA TO DEGREES
    pitch = pitch + (GyY * dt / 1000000.);
    yaw = yaw + (GyZ * dt / 1000000.);

    DEBUG_PRINT("END IMU read");
}
