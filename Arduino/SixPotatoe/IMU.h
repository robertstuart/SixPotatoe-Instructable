#ifndef _IMU_H_
#define _IMU_H_

#include "ICM_20948.h"

const float GYRO_WEIGHT = 0.997;

// Base
class IMU
{
private:
  ICM_20948_I2C icm20948;

  float RANGE_MAX = 2.0;
  void compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ);
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
  void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
  void setSampleRate(float smplFreq);
  void checkDrift(float gyroPitchDelta, float gyroRollDelta, float gyroYawDelta);
  void setDrift(float pitchAve, float rollAve, float yawAve);
  void checkError(const char* s);
  
  bool isError = false;
  const static int DRIFT_SIZE = 100;  // 1/2 second of data
  int aveTotal = 0;  // Total of gyro averages taken.
  float pitchArray[DRIFT_SIZE];
  float rollArray[DRIFT_SIZE];
  float yawArray[DRIFT_SIZE];
  float timeDriftPitch = 0.0;
  float timeDriftRoll = 0.0;
  float timeDriftYaw = 0.0;

protected:

public:
  IMU(); // Constructor

  void imuInit(TwoWire &wirePort, uint8_t ad0_val);
  boolean isNewImuData();
  void printScaledAGMT();
  
  float maPitch = 0;
  float maRoll = 0.0;
  float maYaw = 0.0;
  
  float gyroPitchDelta = 0;
  float gaPitch = 0;
  float gHeading = 0;
  float gaRoll = 0;
  float accelX = 0.0;  
  float accelY = 0.0; 
  float accelZ = 0.0;

  float vertAccel = 0.0;
  float horAccel = 0.0;
  float horAccelFps = 0.0;

};

#endif /* _IMU_H_ */
