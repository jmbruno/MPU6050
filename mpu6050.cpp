/* V0.0.1
 * RTC utilitaires Jean Marc BRUNO / MaYIoT
 * du 06/04/2020
 * *************************************************************************************/

#include <mpu6050.hpp>

int16_t aX = 0, aY = 0, aZ = 0;
int16_t gX = 0, gY = 0, gZ = 0;
int16_t temp = 0;
//=======================================================================================

//
//---------------------------------------------------------------------------------------
bool mpu6050::startMPU(uint8_t adress, uint8_t sda, uint8_t scl, uint32_t clock, bool beg) {
  bool status = true;

  if (beg) status = Wire.begin(sda, scl, clock);
  Wire.beginTransmission(MPU_ADR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  if (Wire.endTransmission(true) == 0) status = true; else status = false;

  if (status) {
#if _DUSB>= 1
    Serial.println("MPU6050 device Initialized...");
#endif
    return true;
  } else {
#if _DUSB>= 1
    Serial.println("MPU6050 device connection failed...");
#endif
    return false;
  }
}
//---------------------------------------------------------------------------------------

// Read MPU6050 Registers
// Parameters:
// - start: start Register
// - nr: Number of register to read (1 to 14)
// Return:  true if good
//---------------------------------------------------------------------------------------
bool mpu6050::readMPU(uint8_t start, uint8_t nr) {
  uint16_t aXl = 0, aYl = 0, aZl = 0, tempL = 0;
  uint16_t aXh = 0, aYh = 0, aZh = 0, tempH = 0;
  uint16_t gXl = 0, gYl = 0, gZl = 0;
  uint16_t gXh = 0, gYh = 0, gZh = 0;
  uint8_t read = start + nr;
  bool status = false;

  Wire.beginTransmission(MPU_ADR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) == 0) status = true; else return false;
  Wire.requestFrom(MPU_ADR, 14, 1);  // request a total of nr registers

  if (nr < 1 || nr > 14) {
#if _DUSB>=1
    Serial.println("Number Register not good");
    status = false;
#endif

    return false;
  }

  switch (start) {
    case ACCEL_XOUT_H: aXh  = Wire.read(); if (read < ACCEL_XOUT_L) break;  // (ACCEL_XOUT_H)
    case ACCEL_XOUT_L: aXl  = Wire.read(); if (read < ACCEL_YOUT_H) break;  // (ACCEL_XOUT_L)
    case ACCEL_YOUT_H: aYh  = Wire.read(); if (read < ACCEL_YOUT_L) break;  // (ACCEL_YOUT_H)
    case ACCEL_YOUT_L: aYl  = Wire.read(); if (read < ACCEL_ZOUT_H) break;  // (ACCEL_YOUT_L)
    case ACCEL_ZOUT_H: aZh  = Wire.read(); if (read < ACCEL_ZOUT_L) break;  // (ACCEL_ZOUT_H)
    case ACCEL_ZOUT_L: aZl  = Wire.read(); if (read < TEMP_OUT_H)   break;  // (ACCEL_ZOUT_L)
    case TEMP_OUT_H  : tempH= Wire.read(); if (read < TEMP_OUT_L)   break;  // (TEMP_OUT_H)
    case TEMP_OUT_L  : tempL= Wire.read(); if (read < GYRO_XOUT_H)  break;  // (TEMP_OUT_L)
    case GYRO_XOUT_H : gXh  = Wire.read(); if (read < GYRO_XOUT_L)  break;  // (GYRO_XOUT_H)
    case GYRO_XOUT_L : gXl  = Wire.read(); if (read < GYRO_YOUT_H)  break;  // (GYRO_XOUT_L)
    case GYRO_YOUT_H : gYh  = Wire.read(); if (read < GYRO_YOUT_L)  break;  // (GYRO_YOUT_H)
    case GYRO_YOUT_L : gYl  = Wire.read(); if (read < GYRO_ZOUT_H)  break;  // (GYRO_YOUT_L)
    case GYRO_ZOUT_H : gZh  = Wire.read(); if (read < GYRO_ZOUT_L)  break;  // (GYRO_ZOUT_H)
    case GYRO_ZOUT_L : gZl  = Wire.read(); break;                           // (GYRO_ZOUT_L)
    default:
#if _DUSB>=1
      Serial.println("Adress not defined");
#endif
      return false;
  }

  aX = aXh << 8 | aXl;
  aY = aYh >> 8 | aYl;
  aY = aZh >> 8 | aZl;
  temp = tempH >> 8 | tempL;
  gX = gXh >> 8 | gXl;
  gY = gYh >> 8 | gYl;
  gZ = gZh >> 8 | gZl;

#if _DUSB>=1
  Serial.printf("- AcX = %d | AcY = %d | AcZ = %d\n", aX, aY, aZ);
  Serial.printf("- GyX = %d | GyY = %d | GuZ = %d\n", gX, gY, gZ);
  Serial.printf("- Temp= %3.1f °C\n", float(temp) / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
#endif

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

mpu6050 mpu;
//=======================================================================================
