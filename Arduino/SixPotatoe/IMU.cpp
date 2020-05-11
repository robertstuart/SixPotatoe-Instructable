/*****************************************************************************-
 *                                 IMU.cpp
 *            Functions to acces the Invensense ICM-20948.
 *         It uses the library from Sparkfun.  The enumerated
 *         values are specified in the following header file:
 *  "Arduino\libraries\SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library\src\util\ICM_20948_ENUMERATIONS.h"
 *            
 *****************************************************************************/
#include "IMU.h"
#define sampleFreq  200.0f      // sample frequency in Hz
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address.
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when
                        // the ADR jumper is closed the value becomes 0

float zeta = 0;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.05f;                              // integration interval for both filter schemes
float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

// Base
IMU::IMU() {

}

/*****************************************************************************-
 *   imuInit()  Initialize the Invensense ICM-20948.  This function use
 *****************************************************************************/
void IMU::imuInit(TwoWire &wirePort, uint8_t ad0_val) {

  bool initialized = false;
  while( !initialized ) {
    // start communication with IMU
    icm20948.begin( wirePort, ad0_val );
    if( icm20948.status != ICM_20948_Stat_Ok ){
      checkError("begin");
      Serial.printf("  Trying again...\n");
      delay(500);
    } else {
      initialized = true;
    }
  }

  // Set sample rate
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = (1125/sampleFreq) - 1;
  mySmplrt.a = (1125/sampleFreq) - 1;
  icm20948.setSampleRate( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, mySmplrt );
  checkError("setSample");

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm8;         // see ICM_20948_ENUMERATIONS.h
  myFSS.g = dps2000;      // see ICM_20948_ENUMERATIONS.h
  icm20948.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  checkError("setFullScale");
  
  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg; // see ICM_20948_ENUMERATIONS.h
  myDLPcfg.a = acc_d23bw9_n34bw4;      // BW = 23, see ICM_20948_ENUMERATIONS.h 
//  myDLPcfg.a = acc_d11bw5_n17bw;       // BW = 11, see ICM_20948_ENUMERATIONS.h 
  myDLPcfg.g = gyr_d119bw5_n154bw3;    // BW = 119, see ICM_20948_ENUMERATIONS.h 
//  myDLPcfg.g = gyr_d51bw2_n73bw3;    // BW = 52, see ICM_20948_ENUMERATIONS.h 
//  myDLPcfg.g = gyr_d23bw9_n35bw9;    // BW = 23, see ICM_20948_ENUMERATIONS.h 
  icm20948.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  checkError((const char*)"setDLPFcfg");
  icm20948.enableDLPF( ICM_20948_Internal_Acc, true );
  checkError("Accelerometer: enableDLPF");
  icm20948.enableDLPF( ICM_20948_Internal_Gyr, true );
  checkError("Gyro: enableDLPF");

  const char* stat = (isError) ? "failed" : "complete";
  Serial.printf("\nICM-20948 configuration %s!\n", stat);
}

void IMU::checkError(const char* s) {
  if( icm20948.status != ICM_20948_Stat_Ok) {
    isError = true;
    Serial.printf("%s() returned: %s\n", icm20948.statusString());
  }
}



/*****************************************************************************-
 *  compFilter()  Complementary filter to get angles
 *****************************************************************************/
void IMU::compFilter(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ) {
  static float gPitch = 0.0;
  static float gRoll = 0.0;

  // Pitch
  gyroPitchDelta = -gyroX / sampleFreq; // degrees changed during period
  gPitch += gyroPitchDelta;   // Debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  float aPitch = ((atan2(-accelY, accelZ)) * RAD_TO_DEG);
  gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));

  // Roll
  float gyroRollDelta = gyroY / sampleFreq;
  gRoll += gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;
  float aRoll =  (atan2(accelX, accelZ) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors

  // Yaw/Heading
  float gyroYawDelta = -gyroZ / sampleFreq; // degrees changed during period
  gHeading += gyroYawDelta;
}



/*****************************************************************************-
     MahonyAHRSupdateIMU()

     From: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 *****************************************************************************/
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 *= recipNorm;
  q[1] = q1 *= recipNorm;
  q[2] = q2 *= recipNorm;
  q[3] = q3 *= recipNorm;
} // End MahonyAHRSupdateIMU()



/*****************************************************************************-
    isNewImuData()   Returns true if the IMU has new data.
                Reads the IMU and sets the new values.
 *****************************************************************************/
boolean IMU::isNewImuData() {

  if (icm20948.dataReady()) {
    icm20948.getAGMT();
    accelX   = icm20948.accX() / 1000;  // divide to get units in g
    accelY   = icm20948.accY() / 1000;  // divide to get units in g
    accelZ   = icm20948.accZ() / 1000;  // divide to get units in g

    float gyroPitchDelta = icm20948.gyrX(); // better to do offset dynamically TODO
    float gyroRollDelta = icm20948.gyrY();
    float gyroYawDelta = icm20948.gyrZ();
    checkDrift(gyroPitchDelta, gyroRollDelta, gyroYawDelta);
    gyroPitchDelta -= timeDriftPitch;
    gyroRollDelta -= timeDriftRoll;
    gyroYawDelta -= timeDriftYaw;
    float gyroXrad = gyroPitchDelta * DEG_TO_RAD;                  // radians/sec
    float gyroYrad = gyroRollDelta * DEG_TO_RAD;                  // radians/sec
    float gyroZrad = gyroYawDelta * DEG_TO_RAD;                  // radians/sec
//    Serial.printf("AG: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", accelX, accelY, accelZ, gyroPitchDelta, gyroRollDelta, gyroYawDelta);

    //    MadgwickQuaternionUpdate(accelX, accelY, accelZ, gyroXrad, gyroYrad, gyroZrad);
    MahonyAHRSupdateIMU(gyroXrad, gyroYrad, gyroZrad, accelX, accelY, accelZ);

    float maPitchRad  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    float maRollRad = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    float maYawRad   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    maPitch = maPitchRad * RAD_TO_DEG;
    maRoll  = maRollRad * RAD_TO_DEG;
    maYaw   = -maYawRad * RAD_TO_DEG;

    compFilter(gyroPitchDelta, gyroRollDelta, gyroYawDelta, accelX, accelY, accelZ);

    vertAccel = (cos(maPitch * DEG_TO_RAD) * accelZ) + (sin(maPitch * DEG_TO_RAD) * accelY);
    horAccel = (sin(maPitch * DEG_TO_RAD) * accelZ) + (cos(maPitch * DEG_TO_RAD) * accelY);
    horAccelFps += horAccel * 0.213; // K3

//    Serial.printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", maPitch, maRoll, maYaw, gaPitch, gaRoll, gHeading);
    return true;
  } else {
    return false; // no IMU read
  }
}



/*****************************************************************************-
 * checkDrift()  Called 200/sec.  Averages gyroDrift for x, y & z
 *              for 1/2 second periods.
 *****************************************************************************/
void IMU::checkDrift(float gyroPitchDelta, float gyroRollDelta, float gyroYawDelta) {
  static int gPtr = 0;
  static float aXSum, aZSum;
  float pitch, roll, yaw;

  pitchArray[gPtr] = gyroPitchDelta;
  rollArray[gPtr] = gyroRollDelta;
  yawArray[gPtr] = gyroYawDelta;
  aXSum += accelX;
  aZSum += accelZ;
  gPtr++;

  if (gPtr >= DRIFT_SIZE) {
    gPtr = 0;
    float pitchSum = 0;
    float rollSum = 0;
    float yawSum = 0;
    float pitchMax = pitchArray[0];
    float pitchMin = pitchMax;
    float rollMax = rollArray[0];
    float rollMin = rollMax;
    float yawMax = yawArray[0];
    float yawMin = yawMax;
    for (int i = 0; i < DRIFT_SIZE; i++) {
      pitch = pitchArray[i];
      if (pitch > pitchMax)  pitchMax = pitch;
      if (pitch < pitchMin)  pitchMin = pitch;
      pitchSum += pitch;
      roll = rollArray[i];
      if (roll > rollMax)  rollMax = roll;
      if (roll < rollMin)  rollMin = roll;
      rollSum += roll;
      yaw = yawArray[i];
      if (yaw > yawMax)  yawMax = yaw;
      if (yaw < yawMin)  yawMin = yaw;
      yawSum += yaw;
    }
    float pitchAve = ((float) pitchSum) / ((float) DRIFT_SIZE);
    float rollAve = ((float) rollSum) / ((float) DRIFT_SIZE);
    float yawAve = ((float) yawSum) / ((float) DRIFT_SIZE);

    // If we have a stable 0.5 second period, average the most recent 20 periods & adjust drift.
    if (((pitchMax - pitchMin) < RANGE_MAX) && ((rollMax - rollMin) < RANGE_MAX) && ((yawMax - yawMin) < RANGE_MAX)) {
      setDrift(pitchAve, rollAve, yawAve);
    }
//    Serial.printf("pitchMin: %4.1f     pitchMax: %4.1f     pitchAve: %5.1f   ", pitchMin, pitchMax, pitchAve);
//    Serial.printf("rollMin: %4.1f     rollMax: %4.1f     rollAve: %5.2f   ", rollMin, rollMax, rollAve);
//    Serial.printf("yawMin: %4.1f     yawMax: %4.1f     yawAve: %5.1f\n", yawMin, yawMax, yawAve);
//    Serial.printf("pitch: %7.4f    roll: %7.4f    yaw: %7.4f\n", pitchMax - pitchMin, rollMax - rollMin, yawMax - yawMin);
  }
}



/*****************************************************************************-
 *    setDrift()  Called to add the last 1/2 sec of drift values.
 *****************************************************************************/
void IMU::setDrift(float pitchAve, float rollAve, float yawAve) {
  static const int AVE_SIZE =  20;      // Equals 10 seconds of measurements
  static float pitchAveArray[AVE_SIZE];
  static float rollAveArray[AVE_SIZE];
  static float yawAveArray[AVE_SIZE];
  static int avePtr = 0;

  float sumPitchAve = 0.0;
  float sumRollAve = 0.0;
  float sumYawAve = 0.0;

  pitchAveArray[avePtr] = pitchAve;
  rollAveArray[avePtr] = rollAve;
  yawAveArray[avePtr] = yawAve;

  ++avePtr;
  avePtr = avePtr % AVE_SIZE;
  if (aveTotal < avePtr) aveTotal = avePtr;

  for (int i = 0; i < aveTotal; i++) {
    sumPitchAve += pitchAveArray[i];
    sumRollAve += rollAveArray[i];
    sumYawAve += yawAveArray[i];
  }
  float avePitchDrift = sumPitchAve / aveTotal;
  float aveRollDrift = sumRollAve / aveTotal;
  float aveYawDrift = sumYawAve / aveTotal;
  timeDriftPitch = avePitchDrift;
  timeDriftRoll = aveRollDrift;
  timeDriftYaw = aveYawDrift;
//  Serial.printf("pitch: %7.4f    roll: %7.4f    yaw: %7.4f\n", timeDriftPitch, timeDriftRoll, timeDriftYaw);
}
