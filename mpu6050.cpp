/* MPU6050 library
 * Based on InvenSense MPU-6050 register map document rev. 4.0, 03/09/2012
 * For Wire library v1.0.1+
 * Le 06 April 2020 by Jean, Marc BRUNO  <jean.marc@bruno.yt>
 * Updates should (hopefully) always be available at https://github.com/jmbruno/MPU6050
 ****************************************************************************************
 * v0.0.1 du 06/04/2020 Initial Commit
 * V0.0.2 du 12/04/2020 Add new functions
 * V0.0.3 du 13/04/2020 New functions and optimisation and bugs corrections
 * *************************************************************************************/

#include <mpu6050.hpp>

uint8_t MPUadress = 0;
int16_t dataMPU[32];
float temp = 0;
struct Gyros gyro;
struct Accelero accel;
//=======================================================================================

// Initialise MPU
//---------------------------------------------------------------------------------------
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
}//--------------------------------------------------------------------------------------

// Find MPU id
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
}//--------------------------------------------------------------------------------------

// Write length bits at bitStart in MPU register
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeBits(uint8_t regAdr, uint8_t bitStart, uint8_t length, uint8_t data) {
  uint8_t mask, content = 0;

  if (readByte(regAdr) != 0) {
    mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);                   // shift data
        data &= mask;                                       //
        content &= ~(mask);                                 // zero all bits used
        content |= data;                                    // combine with existing byte read

        return writeByte(regAdr, content);
    } else {
        return false;
    }
}//--------------------------------------------------------------------------------------

// Write a byte in MPU register
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeByte(uint8_t regAdr, uint8_t data) {
  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);
  Wire.write(data);

  return Wire.endTransmission();
}//--------------------------------------------------------------------------------------

// Write a Word in MPU register
//---------------------------------------------------------------------------------------
uint8_t mpu6050::writeWord(uint8_t regAdr, int16_t data) {
  Wire.beginTransmission(MPUadress);
  Wire.write(regAdr);

  return Wire.endTransmission();
}//--------------------------------------------------------------------------------------

// Write more bytes in MPU register
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
}//--------------------------------------------------------------------------------------

// Read more bytes in MPU register
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
}//--------------------------------------------------------------------------------------

// Read a byte in MPU register
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
}//--------------------------------------------------------------------------------------

// Read a word in MPU register
//---------------------------------------------------------------------------------------
int16_t mpu6050::readWord(uint8_t regAdr) {
  uint8_t status = 0;
  int16_t data = 0;

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
}//--------------------------------------------------------------------------------------

// Read more words in MPU register
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
}//--------------------------------------------------------------------------------------


// Read MPU6050 Registers
// Parameters:
// - start: start register
// - nr: Number of register 16 bits to read (1 to 7)
// Return:  true if good
//---------------------------------------------------------------------------------------
bool mpu6050::readWordsMPU(uint8_t start, uint8_t nr) {
  uint8_t status = false;

  if (nr < 1 || nr > 7) {
#if _DUSB>=1
    Serial.println("Number Register to read not good");
    status = false;
#endif

    return status;
  }

  readWords(ACCEL_XOUT_H, 7, dataMPU, (uint16_t)1000);

  return status;
}//--------------------------------------------------------------------------------------

// Read accelerometers Raw for a scale
//---------------------------------------------------------------------------------------
Accelero mpu6050::getAccelero(uint8_t scale) {
  setScaleAccel(scale);
  accel.X = (float)mpu.readWord(ACCEL_XOUT_H);
  accel.Y = (float)mpu.readWord(ACCEL_YOUT_H);
  accel.Z = (float)mpu.readWord(ACCEL_ZOUT_H);

  return accel;
}//--------------------------------------------------------------------------------------

// Read Gravitys in g for a scale
//---------------------------------------------------------------------------------------
Accelero mpu6050::getGravity(uint8_t scale) {
  float sensibility = 0;

  setScaleAccel(scale);

  switch (scale) {
    case AFS_SEL_2:  sensibility = 16384.0; break;
    case AFS_SEL_4:  sensibility =  8192.0; break;
    case AFS_SEL_8:  sensibility =  4096.0; break;
    case AFS_SEL_16: sensibility =  2048.0; break;
    default:         sensibility =     1.0; break;
  }

  accel.X = (float)mpu.readWord(ACCEL_XOUT_H) / sensibility;
  accel.Y = (float)mpu.readWord(ACCEL_YOUT_H) / sensibility;
  accel.Z = (float)mpu.readWord(ACCEL_ZOUT_H) / sensibility;

  return accel;
}//--------------------------------------------------------------------------------------

// Read Rotations Raw for a scale
//---------------------------------------------------------------------------------------
Gyros mpu6050::getGyros(uint8_t scale) {
  setScaleGyros(scale);
  gyro.X = (float)mpu.readWord(GYRO_XOUT_H);
  gyro.Y = (float)mpu.readWord(GYRO_YOUT_H);
  gyro.Z = (float)mpu.readWord(GYRO_ZOUT_H);

  return gyro;
}//--------------------------------------------------------------------------------------

// Read Rotations in °/s for a scale
//---------------------------------------------------------------------------------------
Gyros mpu6050::getDegperSecs(uint8_t scale) {
  float sensibility = 0;

  setScaleGyros(scale);
  switch (scale) {
    case GFS_SEL_250:  sensibility = 131.0; break;
    case GFS_SEL_500:  sensibility =  65.5; break;
    case GFS_SEL_1000: sensibility =  32.8; break;
    case GFS_SEL_2000: sensibility =  16.4; break;
    default:           sensibility =   1.0; break;
  }
  gyro.X = (float)mpu.readWord(GYRO_XOUT_H) / sensibility;
  gyro.Y = (float)mpu.readWord(GYRO_YOUT_H) / sensibility;
  gyro.Z = (float)mpu.readWord(GYRO_ZOUT_H) / sensibility;

  return gyro;
}//--------------------------------------------------------------------------------------

// Read Temperature in °C
//---------------------------------------------------------------------------------------
float mpu6050::getTemperature() {
  //Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
  return (float)mpu.readWord(TEMP_OUT_H)/340 + 36.53;
}//--------------------------------------------------------------------------------------

// Set accelerometer scale
//---------------------------------------------------------------------------------------
void mpu6050::setScaleAccel(uint8_t range) {
  writeBits(ACCEL_CONFIG, CONF_AFS_SEL_BIT, CONF_AFS_SEL_LENGTH, range);
}//--------------------------------------------------------------------------------------

// Set Gyroscope scale
//---------------------------------------------------------------------------------------
void mpu6050::setScaleGyros(uint8_t range) {
  writeBits(GYRO_CONFIG, CONF_GFS_SEL_BIT, CONF_GFS_SEL_LENGTH, range);
}//--------------------------------------------------------------------------------------


/* -gyro_lpf Gyroscope low pass filter setting,
 *   7: 8kHz sampling rate, 36001Hz bandwidth,  0.17ms delay.
 *   0: 8kHz sampling rate,   250Hz bandwidth,  0.97ms delay.
 *   1: 1kHz sampling rate,   184Hz bandwidth,  2.90ms delay.
 *   2: 1kHz sampling rate,    92Hz bandwidth,  3.90ms delay.
 *   3: 1kHz sampling rate,    41Hz bandwidth,  5.90ms delay.
 *   4: 1kHz sampling rate,    20Hz bandwidth,  9.90ms delay.
 *   5: 1kHz sampling rate,    10Hz bandwidth, 17.85ms delay.
 *   6: 1kHz sampling rate,     5Hz bandwidth, 33.48ms delay.
 * -accel_lpf Accelerometer low pass filter setting,
 *   7: 1kHz sampling rate, 420.0Hz 3dB bandwidth,  1.38ms delay.
 *   0: 1kHz sampling rate, 218.1Hz 3dB bandwidth,  1.88ms delay.
 *   1: 1kHz sampling rate, 218.1Hz 3dB bandwidth,  1.88ms delay.
 *   2: 1kHz sampling rate,  99.0Hz 3dB bandwidth,  2.88ms delay.
 *   3: 1kHz sampling rate,  44.8Hz 3dB bandwidth,  4.88ms delay.
 *   4: 1kHz sampling rate,  21.2Hz 3dB bandwidth,  8.87ms delay.
 *   5: 1kHz sampling rate,  10.2Hz 3dB bandwidth, 16.83ms delay.
 *   6: 1kHz sampling rate,  5.05Hz 3dB bandwidth, 32.48ms delay.
 *-------------------------------------------------------------------------------------*/
// Set Low pass filter for both the gyroscopes and accelerometers.
//---------------------------------------------------------------------------------------
void mpu6050::setFilter(uint8_t bandwidth) {
  writeBits(DLPF_CONFIG, DLPF_CFG_BIT, DLPF_CFG_LENGTH, bandwidth);
}//--------------------------------------------------------------------------------------

// Compute and set Gyroscopes offset
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
}//--------------------------------------------------------------------------------------

mpu6050 mpu;

//=======================================================================================
