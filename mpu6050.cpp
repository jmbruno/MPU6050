/* V0.0.2
 * RTC utilitaires Jean Marc BRUNO / MaYIoT
 * du 06/04/2020
 * For Wire library v1.0.1+
 * *************************************************************************************/

#include <mpu6050.hpp>

uint8_t MPUadress = 0;
int16_t dataMPU[32];
int16_t aX = 0, aY = 0, aZ = 0;
int16_t gX = 0, gY = 0, gZ = 0;
int16_t temp = 0;
//=======================================================================================

/**
 * Reset the MPU and configure the gyroscope to +- 2000 degrees/second and the accelerometer to +-16g.
 * The low pass filter settings control how sensitive the sensors are to quick changes.
 * In order of increasing sensitivity: 6, 5, 4, 3, 2, 1, 0, 7
 * Info from MPU-9250 register map document.
 * -gyro_lpf Gyroscope low pass filter setting,
 *   7: 8kHz sampling rate, 36001Hz bandwidth, 0.17ms delay.
 *   0: 8kHz sampling rate, 250Hz bandwidth, 0.97ms delay.
 *   1: 1kHz sampling rate, 184Hz bandwidth, 2.9ms delay.
 *   2: 1kHz sampling rate, 92Hz bandwidth, 3.9ms delay.
 *   3: 1kHz sampling rate, 41Hz bandwidth, 5.9ms delay.
 *   4: 1kHz sampling rate, 20Hz bandwidth, 9.9ms delay.
 *   5: 1kHz sampling rate, 10Hz bandwidth, 17.85ms delay.
 *   6: 1kHz sampling rate, 5Hz bandwidth, 33.48ms delay.
 * -accel_lpf Accelerometer low pass filter setting,
 *   7: 1kHz sampling rate, 420Hz 3dB bandwidth, 1.38ms delay.
 *   0: 1kHz sampling rate, 218.1Hz 3dB bandwidth, 1.88ms delay.
 *   1: 1kHz sampling rate, 218.1Hz 3dB bandwidth, 1.88ms delay.
 *   2: 1kHz sampling rate, 99Hz 3dB bandwidth, 2.88ms delay.
 *   3: 1kHz sampling rate, 44.8Hz 3dB bandwidth, 4.88ms delay.
 *   4: 1kHz sampling rate, 21.2Hz 3dB bandwidth, 8.87ms delay.
 *   5: 1kHz sampling rate, 10.2Hz 3dB bandwidth, 16.83ms delay.
 *   6: 1kHz sampling rate, 5.05Hz 3dB bandwidth, 32.48ms delay.
 *-------------------------------------------------------------------------------------*/
bool mpu6050::startMPU(uint8_t I2Cadr, uint8_t sda, uint8_t scl, uint32_t clock, bool beg) {
  bool status = true;

  if (beg) status = Wire.begin(sda, scl, clock);
  Wire.beginTransmission(I2Cadr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  if (Wire.endTransmission(true) == 0) status = true; else status = false;

  if (status) {
#if _DUSB>= 1
    Serial.printf("MPU6050 device Initialized at 0x%02X Adress...\n", I2Cadr);
#endif
    MPUadress = I2Cadr;

    return true;
  } else {
#if _DUSB>= 1
    Serial.println("MPU6050 device connection failed...");
#endif
    return false;
  }
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
String mpu6050::findMPU() {
  int8_t sensorId;

  sensorId = readByte(WHO_AM_I);
#if _DUSB>=1
  Serial.printf("***** MPU Id: 0x%02x\n", sensorId);
#endif

  switch(sensorId) {
    case MPU_6050_ID: return "MPU-6050";
    case MPU_6500_ID: return "MPU-6500";
    case MPU_9250_ID: return "MPU-9250";
    case MPU_9255_ID: return "MPU-9255";
    default:          return "--------";
  }
}
//--------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeByte(uint8_t regAdr, uint8_t data) {
  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  Wire.write(data);

  return Wire.endTransmission();
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeWord(uint8_t regAdr, int16_t data) {
  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);

  return Wire.endTransmission();
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeBytes(uint8_t regAdr, uint8_t length, int8_t *data) {
  uint8_t status = 0;

  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  if (length != 0) {
    for (uint8_t i= 0; i < length; i++) {
      Wire.write(data[i]);
    }
  }
  status = Wire.endTransmission();

  return status == 0;
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
uint8_t mpu6050::readBytes(uint8_t regAdr, uint8_t length, int8_t *data, uint16_t timeOut) {
  uint8_t status = 0, count = 0;
  uint32_t t1 = millis();

#if _DUSB>=2
  Serial.printf("I2C (0x%0x) reading %u bytes from 0x%02X...\n", I2Caddr, length, regAddr);
#endif

  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  status = Wire.endTransmission();

  for (uint8_t k= 0; k < length; k += min(length, BUF_LENGTH)) {
    //status = writeByte(regAddr, 0);
    if (status != 0) return status;

    Wire.beginTransmission(MPUadress);
    Wire.requestFrom(MPUadress, (uint8_t)__min(length, BUF_LENGTH));

    for (; Wire.available() && (timeOut == 0 || millis() - t1 < timeOut); count++) {
      data[count] = Wire.read();
#if _DUSB>=1
      Serial.printf("%02X", data[count]);
      if ((count + 1) < length) Serial.print(" ");
#endif
    }
    Wire.endTransmission();
  }

  if (timeOut > 0 && millis() - t1 >= timeOut && count < length) count = -1; // timeout

  return count;
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
int8_t mpu6050::readByte(uint8_t regAdr) {
  int8_t status = 0;
  int8_t data = 0;
  //uint32_t t1 = millis();

  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  status = Wire.endTransmission();
  if (status != 0) return -1;

  Wire.requestFrom(MPUadress, (uint8_t)1);

  data = Wire.read();

#if _DUSB>=2
  Serial.printf("%02X", data);
#endif

   Wire.endTransmission();

  return data;
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
int16_t mpu6050::readWord(uint8_t regAdr) {
  uint8_t status = 0;
  int16_t data = 0;
  //uint32_t t1 = millis();

  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  status = Wire.endTransmission();
  if (status != 0) return -1;

  Wire.requestFrom(MPUadress, (uint8_t)2);

  data = Wire.read() << 8;
  data = data | Wire.read();

#if _DUSB>=2
  Serial.printf("%04X", data);
#endif

   Wire.endTransmission();

  return data;
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
uint8_t mpu6050::readWords(uint8_t regAdr, uint8_t length, int16_t *data, uint16_t timeOut) {
  bool msb = true;
  int8_t status = 0, count = 0;
  uint32_t t1 = millis();

  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  status = Wire.endTransmission();

  for (uint8_t k= 0; k < length * 2; k += __min(length * 2, BUF_LENGTH)) {
    Wire.requestFrom(MPUadress, (uint8_t)__min(length * 2, BUF_LENGTH));

    for (; Wire.available() && count < length && (timeOut == 0 || millis() - t1 < timeOut); ) {
      if (msb) {
        data[count] = Wire.read() << 8;
        count++;
      } else {
        data[count] = Wire.read();
        count++;
      }
      msb = !msb;
#if _DUSB>=2
      Serial.printf("%04X", data[count]);
      if (count + 1 < length) Serial.print(" ");
#endif
    }
    Wire.endTransmission();
  }

  if (timeOut > 0 && millis() - t1 >= timeOut && count < length) status = -1; else status = count; // timeout

  return status;
}
//---------------------------------------------------------------------------------------


// Read MPU6050 Registers
// Parameters:
// - start: start Register
// - nr: Number of register to read (1 to 14)
// Return:  true if good
//---------------------------------------------------------------------------------------
bool mpu6050::readMPU(uint8_t start, uint8_t nr) {
  uint8_t status = false;

  //status = writeByte(MPUadress, ACCEL_XOUT_H);

  if (nr < 1 || nr > 7) {
#if _DUSB>=1
    Serial.println("Number Register to read not good");
    status = false;
#endif

    return status;
  }

  readWords(ACCEL_XOUT_H, 7, dataMPU, (uint16_t)1000);

  return status;
}
//---------------------------------------------------------------------------------------

/*/
//---------------------------------------------------------------------------------------
void getAcceleration(float scale) {
  mpu6050::readMPU(ACCEL_XOUT_H, 6);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void getRotation(float scale) {
  mpu6050::readMPU(GYRO_XOUT_H, 6);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void getGravity(float scale) {
  mpu6050::readMPU(ACCEL_XOUT_H, 6);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void getDegperSecs(float scale) {
  mpu6050::readMPU(GYRO_XOUT_H, 6);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void XsetGyroOffset(uint16_t offsset) {

}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void YsetGyroOffset(uint16_t offsset) {

}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void ZsetGyroOffset(uint16_t offsset) {

}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void ZAcceleroOffset(uint16_t offsset) {

}
//---------------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------------
void AcceleroCalibre(uint16_t calTime) {

}
//---------------------------------------------------------------------------------------*/

//
//---------------------------------------------------------------------------------------
void mpu6050::setScaleAccel(uint8_t range) {
  writeByte(ACCEL_CONFIG, range);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
void mpu6050::setScaleGyros(uint8_t range) {
  writeByte(GYRO_CONFIG, range);
}
//---------------------------------------------------------------------------------------

//
//---------------------------------------------------------------------------------------
boolean mpu6050::computeGyroOffset(uint8_t moy) {
  uint16_t OfgX = 0, OfgY = 0, OfgZ = 0;

  writeWord(GX_OFFSET_H, 0);
  writeWord(GY_OFFSET_H, 0);
  writeWord(GZ_OFFSET_H, 0);

  for (uint8_t i= 0; i < moy; i++) {
    OfgX += readWord(GYRO_XOUT_H);
    OfgY += readWord(GYRO_YOUT_H);
    OfgZ += readWord(GYRO_ZOUT_H);
    delay(10);
  }

  OfgX = OfgX / moy;
  OfgY = OfgY / moy;
  OfgZ = OfgZ / moy;
  if (OfgX > MAX_VAL || OfgY > MAX_VAL || OfgZ > MAX_VAL) {
    return false;
  }

  writeWord(GX_OFFSET_H, -2 * OfgX);
  writeWord(GY_OFFSET_H, -2 * OfgY);
  writeWord(GZ_OFFSET_H, -2 * OfgZ);

  return true;
}
//---------------------------------------------------------------------------------------
mpu6050 mpu;
//=======================================================================================
