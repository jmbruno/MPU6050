/* MPU6050 library
 * Based on InvenSense MPU-6050 register map document rev. 4.0, 06/09/2012
 * Le 06 April 2020 by Jean, Marc BRUNO  <jean.marc@bruno.yt>
 * Updates should (hopefully) always be available at https://github.com/jmbruno/MPU6050
 ***************************************************************************************/

#ifndef MPU_6050_HPP
#define MPU_6050_HPP

#include <Arduino.h>
#include <Wire.h>

#define __min(a,b) ((a)<(b)?(a):(b))
//=======================================================================================

const uint8_t MPU_6050_ADR      = 0x68;      // Adresse du MPU6050
const uint8_t MPU_6050_ID       = 0x68;
const uint8_t MPU_6500_ID       = 0x70;
const uint8_t MPU_9250_ID       = 0x71;
const uint8_t MPU_9255_ID       = 0x73;
const uint8_t SIGNAL_PATH_RESET = 0x68;
const uint8_t REG_USER_CTRL     = 0x6a;
const uint8_t PWR_MGMT_1        = 0x6B;      // PWR_MGMT_1 register
const uint8_t WHO_AM_I          = 0x75;

const uint8_t ACCEL_XOUT_H      = 0x3B;
const uint8_t ACCEL_XOUT_L      = 0x3C;
const uint8_t ACCEL_YOUT_H      = 0x3D;
const uint8_t ACCEL_YOUT_L      = 0x3E;
const uint8_t ACCEL_ZOUT_H      = 0x3F;
const uint8_t ACCEL_ZOUT_L      = 0x40;
const uint8_t TEMP_OUT_H        = 0x41;
const uint8_t TEMP_OUT_L        = 0x42;
const uint8_t GYRO_XOUT_H       = 0x43;
const uint8_t GYRO_XOUT_L       = 0x44;
const uint8_t GYRO_YOUT_H       = 0x45;
const uint8_t GYRO_YOUT_L       = 0x46;
const uint8_t GYRO_ZOUT_L       = 0x47;
const uint8_t GYRO_ZOUT_H       = 0x48;

const uint8_t DLPF_CONFIG         = 0x1A;
const uint8_t DLPF_CFG_BIT        = 2;         // Start bit in register
const uint8_t DLPF_CFG_LENGTH     = 3;         // Length bits to write or read
const uint8_t GYRO_CONFIG         = 0x1B;
const uint8_t CONF_GFS_SEL_BIT    = 4;     //
const uint8_t CONF_GFS_SEL_LENGTH = 2;     //
const uint8_t ACCEL_CONFIG        = 0x1C;
const uint8_t CONF_AFS_SEL_BIT    = 4;     //
const uint8_t CONF_AFS_SEL_LENGTH = 2;     //
const uint8_t ACCEL_CONFIG2       = 0x1D;
const uint8_t CONF_HPF_BIT        = 2;     //
const uint8_t CONF_HPF_LENGTH     = 3;     //

const uint8_t GX_OFFSET_H       = 0x13;      // Offset X Gyroscope
const uint8_t GY_OFFSET_H       = 0x15;      // Offset Y Gyroscope
const uint8_t GZ_OFFSET_H       = 0x17;      // Offset Z Gyroscope
const uint8_t GFS_SEL_250       = 0;         // Calibre +-250°/s Gyrosope
const uint8_t GFS_SEL_500       = 1;         // Calibre +-500°/s Gyrosope
const uint8_t GFS_SEL_1000      = 2;         // Calibre +-1000°/s Gyrosope
const uint8_t GFS_SEL_2000      = 3;         // Calibre +-2000°/s Gyrosope
const uint8_t AFS_SEL_2         = 0;         // Calibre +-2g Accelero
const uint8_t AFS_SEL_4         = 1;         // Calibre +-4g Accelero
const uint8_t AFS_SEL_8         = 2;         // Calibre +-8g Accelero
const uint8_t AFS_SEL_16        = 3;         // Calibre +-16g Accelero

                                             //           |   ACCELEROMETER  |  GYROSCOPE
const uint8_t DLPF_BW_256       = 0x00;      // 260Hz     | 0ms    | 256Hz   | 0.98ms | 8kHz
const uint8_t DLPF_BW_188       = 0x01;      // 184Hz     | 2.0ms  | 188Hz   | 1.9ms  | 1kHz
const uint8_t DLPF_BW_98        = 0x02;      // 94Hz      | 3.0ms  | 98Hz    | 2.8ms  | 1kHz
const uint8_t DLPF_BW_42        = 0x03;      // 44Hz      | 4.9ms  | 42Hz    | 4.8ms  | 1kHz
const uint8_t DLPF_BW_20        = 0x04;      // 21Hz      | 8.5ms  | 20Hz    | 8.3ms  | 1kHz
const uint8_t DLPF_BW_10        = 0x05;      // 10Hz      | 13.8ms | 10Hz    | 13.4ms | 1kHz
const uint8_t DLPF_BW_5         = 0x06;      // 5Hz       | 19.0ms | 5Hz     | 18.6ms | 1kHz

const uint8_t BUF_LENGTH        = 32;
const uint16_t MAX_VAL          = 60;        // stabilite de l'offset
const uint16_t MAX_CAL          = 65535 /2;  // Value of max. register

struct Accelero {
  float X = 0;
  float Y = 0;
  float Z = 0;
};
struct Gyros {
  float X = 0;
  float Y = 0;
  float Z = 0;
};
extern struct Accelero accel;
extern struct Gyros gyro;
extern float temp;
extern int16_t dataMPU[32];
extern uint8_t MPUadress;
//---------------------------------------------------------------------------------------

class mpu6050 {

  public:

    bool startMPU(uint8_t I2Cadr, uint8_t sda, uint8_t scl, uint32_t clock, bool beg);
    String findMPU();
    int16_t readWord(uint8_t regAdr);
    bool readWordsMPU(uint8_t start, uint8_t nr);
    void setScaleAccel(uint8_t range);
    void setScaleGyros(uint8_t range);
    void setFilter(uint8_t bandwidth);
    boolean computeGyroOffset(uint8_t moy);
    Accelero getGravity(uint8_t scale);
    Gyros getDegperSecs(uint8_t scale);
    float getTemperature();

  private:
    int8_t  readByte(uint8_t regAdr);
    uint8_t readBytes(uint8_t regAdr, uint8_t length, int8_t  *data, uint16_t timeOut);
    uint8_t readWords(uint8_t regAdr, uint8_t length, int16_t *data, uint16_t timeOut);
    uint8_t writeBits(uint8_t regAdr, uint8_t startBit, uint8_t length, uint8_t data);
    uint8_t writeByte(uint8_t regAdr, uint8_t data);
    uint8_t writeWord(uint8_t regAdr, int16_t data);
    uint8_t writeBytes(uint8_t regAdr, uint8_t length, int8_t *data);
    boolean readRegister(uint8_t adress, uint8_t regist);
    boolean writeRegister(uint8_t adress, uint8_t regist);
    Gyros getGyros(uint8_t scale);
    Accelero getAccelero(uint8_t scale);
};

extern mpu6050 mpu;

#endif
//***************************************************************************************
