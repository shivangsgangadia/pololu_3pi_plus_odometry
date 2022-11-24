#ifndef __IMU_UTILS_H
#define __IMU_UTILS_H

#include <LSM6.h>
#include <LIS3MDL.h>

#define IMU_GYRO_DEFAULT_CONST 8.75
#define MAG_SENSITIVITY 4

namespace GYROSCOPE {
  int gyro_z_calibration_value;

  void calibrateGyroscope(LSM6 *imu) {
    // IMU calibration routine
    long tempZ = 0;
    int rep_count = 0, init_time = millis();
    while (millis() - init_time < 3000) {
      imu->read();
      tempZ += (imu->g.z * IMU_GYRO_DEFAULT_CONST);
      rep_count++;
    }
    GYROSCOPE::gyro_z_calibration_value = tempZ / rep_count;
    Serial.print("Calibration Value : \t");
    Serial.println(gyro_z_calibration_value);
  }

  float gz_temp, gz_prev = 0, gz_bias = 0.8;

  float getGyroReading(int timePassed, LSM6 *imu) {
    imu->read();
    gz_temp = gz_bias * (((imu->g.z * IMU_GYRO_DEFAULT_CONST)) - GYROSCOPE::gyro_z_calibration_value) + (1-gz_bias) * (gz_prev); // Reading in milli degrees per second with a low pass filter
    gz_temp = gz_temp / (1000 * 10000);
    gz_prev = gz_temp;
    return (gz_temp * timePassed);
  }

}

namespace MAGNETOMETER {
  struct {
    int offsetX, offsetY, offsetZ;
    int scaleFactorX, scaleFactorY, scaleFactorZ;
    int averageRange;
  } MAG_CALIBRATION_DATA;

  void calibrateMagnetometer(LIS3MDL *mag) {
    digitalWrite(YELLOW_PIN, HIGH);
    delay(1000);
    int maxX = 0, maxY = 0, maxZ = 0,
        minX = -32000, minY = -32000, minZ = -32000,
        initTime = millis();
    while (millis() - initTime <= 4000) {
      mag->read();
      if (mag->m.x > maxX) maxX = mag->m.x;
      if (mag->m.x < minX) minX = mag->m.x;

      if (mag->m.y > maxY) maxY = mag->m.y;
      if (mag->m.y < minY) minY = mag->m.y;

      if (mag->m.z > maxZ) maxZ = mag->m.z;
      if (mag->m.z < minZ) minZ = mag->m.z;
    }
    MAGNETOMETER::MAG_CALIBRATION_DATA.offsetX = (maxX - minX) / 2;
    MAGNETOMETER::MAG_CALIBRATION_DATA.offsetY = (maxY - minY) / 2;
    MAGNETOMETER::MAG_CALIBRATION_DATA.offsetZ = (maxZ - minZ) / 2;

    int rangeX = (maxX - minX);
    int rangeY = (maxY - minY);
    int rangeZ = (maxZ - minZ);

    MAGNETOMETER::MAG_CALIBRATION_DATA.averageRange = (rangeX + rangeY + rangeZ) / 3;
    MAGNETOMETER::MAG_CALIBRATION_DATA.scaleFactorX = MAG_CALIBRATION_DATA.averageRange / rangeX;
    MAGNETOMETER::MAG_CALIBRATION_DATA.scaleFactorY = MAG_CALIBRATION_DATA.averageRange / rangeY;
    MAGNETOMETER::MAG_CALIBRATION_DATA.scaleFactorZ = MAG_CALIBRATION_DATA.averageRange / rangeZ;

    digitalWrite(YELLOW_PIN, LOW);
    
  }

  float m_x, m_y, mz_degrees, mz_prev, mz_bias = 0.8;
  float getMagnetoReading(LIS3MDL *mag) {
    mag->read();
    m_x = MAGNETOMETER::MAG_CALIBRATION_DATA.scaleFactorX * (mag->m.x) * MAG_SENSITIVITY;
    m_y = MAGNETOMETER::MAG_CALIBRATION_DATA.scaleFactorY * (mag->m.y) * MAG_SENSITIVITY;
    mz_degrees = mz_bias * (atan2(m_y, m_x) * 100) + (1-mz_bias) * (mz_prev);   // magneto low pass filter
    mz_prev = mz_degrees;
    return (mz_degrees);
  }
}

#endif
