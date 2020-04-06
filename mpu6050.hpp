#ifndef MPU_6050_HPP
#define MPU_6050_HPP

#include <Arduino.h>
#include <Wire.h>
//=======================================================================================

const uint8_t MPU_ADR      = 0x68;      // Adresse du MPU6050
const uint8_t PWR_MGMT_1   = 0x6B;      // PWR_MGMT_1 register
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t ACCEL_XOUT_L = 0x3C;
const uint8_t ACCEL_YOUT_H = 0x3D;
const uint8_t ACCEL_YOUT_L = 0x3E;
const uint8_t ACCEL_ZOUT_H = 0x3F;
const uint8_t ACCEL_ZOUT_L = 0x40;
const uint8_t TEMP_OUT_H   = 0x41;
const uint8_t TEMP_OUT_L   = 0x42;
const uint8_t GYRO_XOUT_H  = 0x43;
const uint8_t GYRO_XOUT_L  = 0x44;
const uint8_t GYRO_YOUT_H  = 0x45;
const uint8_t GYRO_YOUT_L  = 0x46;
const uint8_t GYRO_ZOUT_L  = 0x47;
const uint8_t GYRO_ZOUT_H  = 0x48;

extern int16_t aX, aY, aZ;
extern int16_t gX, gY, gZ;
extern int16_t temp;

//---------------------------------------------------------------------------------------
class mpu6050 {

  public :

    bool startMPU(uint8_t adress, uint8_t sda, uint8_t scl, uint32_t clock, bool beg);
    bool readMPU(uint8_t start, uint8_t nr);

  private:

};

extern mpu6050 mpu;

#endif
//***************************************************************************************
