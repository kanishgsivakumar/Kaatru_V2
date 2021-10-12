#define MPU_ADDR 0x68
#include <Wire.h>
#include <Kaatru_sensor.h>
 //******************************
 //*Abstract: Read values of accelration in X,Y & Z axis ,Gyroscope in X,Y & Z axis along with temperature from MPU6050 over I2C 
 //
 //******************************
void mpuInit(TwoWire* Wire){
    Wire -> beginTransmission(MPU_ADDR);
    Wire -> write(0x6B); //PGMT_1 107 register 
    Wire -> write(0); // 0 for Wake up  (0x40 for Device reset)
    Wire -> endTransmission(true);
}
int16_t getAccelx(TwoWire* Wire){
    Wire -> beginTransmission(MPU_ADDR);
    Wire ->write(0x3B); // // starting with register 0x3B (ACCEL_XOUT_H)
    Wire -> endTransmission(false);
    Wire -> requestFrom(MPU_ADDR,2,1);
    int16_t   AcX = Wire -> read() << 8 | Wire -> read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    return AcX;
}

int16_t getAccely(TwoWire* Wire){
    Wire -> beginTransmission(MPU_ADDR);
    Wire ->write(0x3D); // // starting with register 0x3D (ACCEL_YOUT_H)
    Wire -> endTransmission(false);
    Wire -> requestFrom(MPU_ADDR,2,1);
    int16_t   AcY = Wire -> read() << 8 | Wire -> read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_XOUT_L)
    return AcY;
}

int16_t getMputemp(TwoWire* Wire){
    Wire -> beginTransmission(MPU_ADDR);
    Wire ->write(0x41); // // starting with register 0x3D (ACCEL_YOUT_H)
    Wire -> endTransmission(false);
    Wire -> requestFrom(MPU_ADDR,2,1);
    int16_t Tmp = Wire->read() << 8 | Wire->read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    return Tmp;
}
struct imu_data getMotion(TwoWire* Wire){
    
    Wire -> beginTransmission(MPU_ADDR);
    Wire->write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire->endTransmission(false);
    Wire->requestFrom(MPU_ADDR, 14, 1); // request a total of 14 registers
    struct imu_data imu;
    imu.acx = Wire->read() << 8 | Wire->read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    imu.acy = Wire->read() << 8 | Wire->read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    imu.acz = Wire->read() << 8 | Wire->read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    imu.tmp = Wire->read() << 8 | Wire->read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    imu.gcx = Wire->read() << 8 | Wire->read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    imu.gcy = Wire->read() << 8 | Wire->read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    imu.gcz = Wire->read() << 8 | Wire->read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    imu.millis = millis();
    return imu;
}

