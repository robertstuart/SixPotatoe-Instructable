/*****************************************************************************-
 *                           Run.ino 
 *****************************************************************************/
//float kphCorrection = 0.0f;
//float angleError = 0.0;
//float targetPitch = 0.0;
//float kphAdjustment = 0.0;


/*****************************************************************************-
 *  run() Continuous loop for doing all tasks.
 *****************************************************************************/
void run() {
  commonTasks();
  if (imu.isNewImuData()) {
      
    if (isRouteInProgress) routeControl(); 
    else rcControl();

    setCoKph();
    if (isUpright) balance();
      
    runMotors();
    blinkTeensy();
    updateCartesian();
    postLog();
  }
}



/*****************************************************************************-
 *  rcControl() Control speed & steering from RC controller
 *****************************************************************************/
void rcControl() {
  const float Z_TURN = 3.0;
  const float KPH_TC = 0.1;
  static float lpWKph = 0.0; // prevent zero
  float zeroTurn = 0.0;

  balanceTargetKph = controllerY * MAX_MOTOR_KPH * K30;
  lpWKph = (wKph * KPH_TC) + (lpWKph * (1.0 - KPH_TC)); // LP filter  
  if (abs(lpWKph) < 0.01) lpWKph = 0.01;  // can't divide by zero.
  
  float x = constrain(controllerX, -.9999, 0.9999);
  float sign = (x < 0.0) ? 1.0 : -1.0;
  float diff = pow(abs(x), 2.4) * sign;

  // Limit radius at higher speeds.
  float maxDiff = 2.0 / abs(lpWKph); 
  if (diff > maxDiff) diff = maxDiff;

  // Add turn at low speeds so can turn in place
  float absKph = abs(lpWKph);
  if (absKph < Z_TURN) {
    float part = (Z_TURN - absKph) / Z_TURN;
    zeroTurn = part * x * 4.0;
  }
  
  balanceSteerAdjustment = (diff * lpWKph) - zeroTurn;

  // Set values for on the ground (!isUpright)
  float targetWKph = controllerY * K31;
  float steerDiff = controllerX * K32;
  targetWKphRight = targetWKph + steerDiff;
  targetWKphLeft = targetWKph - steerDiff;
}



/*****************************************************************************-
 *  balance() 
 *****************************************************************************/
void balance() {
  static float coKphError = 0.0;
  
  // Find the speed error while limiting the rate of change of this error.
//  if (!isRunning) coKphError = 0.0;
//  float diff = balanceTargetKph - coKph;
//  if      (diff > K4)  coKphError += K4;
//  else if (diff < -K4) coKphError -= K4;
//  else                 coKphError = diff;
  coKphError = balanceTargetKph - coKph;

  // compute a weighted angle to eventually correct the speed error
  float targetPitch = -(coKphError * K5_RESULT); // Angle Gain. Speed error to angle 
  
  // Enforce limits on maximum angle.
  targetPitch = constrain(targetPitch, -K12, K12);

  // Compute angle error and weight factor
  float angleError = targetPitch - imu.maPitch;
  angleError = constrain(angleError, -K13, K13); // prevent "jumping"
  float kphCorrection = angleError * K14_RESULT; // Angle error to speed (Kph) 
  float d = imu.gyroPitchDelta *  K17; // add "D" to reduce overshoot

  // Reduce D to zero at K15
  float ratio = (K15 - abs(imu.maPitch)) / K15;
  ratio = constrain(ratio, 0.0, 1.0);
  kphCorrection -= (d * ratio);

  // Add the angle error to the base speed to get the target wheel speed.
  targetWKph = kphCorrection + coKph;

  targetWKphRight = targetWKph - balanceSteerAdjustment;
  targetWKphLeft = targetWKph + balanceSteerAdjustment;
} // end balance() 



/*****************************************************************************-
 *  setCoKPh() Set the center of oscillation speed (cos) which is determined 
 *             by accelerometer and/or wheel speed.
 *****************************************************************************/
void setCoKph() {
  static float wheelCoKphOld = 0.0;
  static const int COS_BUF_SIZE = 3;
  static float cosBuff[COS_BUF_SIZE];
  static int cosBuffPtr = 0;
  float delta = 0.0;
  float rotation = 0.0;

  isZeroG = (imu.vertAccel < 0.5) ? true : false;
  if (abs(imu.maPitch) > 30.0) isZeroG = false; // Only do when pretty vertical.

  if (isAir) {
    zeroGKph = wheelCoKphOld = wKph;
  } else if (isZeroG) { // Use cos before it went zeroG.
    coKph = zeroGKph;
  } else {  // Compute cos using wheel speed.
    delta = imu.gyroPitchDelta * cos(DEG_TO_RAD * imu.maPitch);
    rotation = delta * K1;  // ~1.6
    float wheelCoKph = wKph - rotation;
    wheelCoKph = (wheelCoKphOld * (1 - K2)) + (wheelCoKph * K2); //  
    wheelCoKphOld = wheelCoKph;
    coKph = wheelCoKph;
    zeroGKph = cosBuff[cosBuffPtr]; // In case falling on next loop.
    cosBuffPtr++;
    cosBuffPtr = cosBuffPtr % COS_BUF_SIZE; 
    cosBuff[cosBuffPtr] = coKph;
  }
}



/*****************************************************************************-
 *  postLog() Called 200 times/sec.
 *****************************************************************************/
void postLog() {
  static unsigned int logLoop = 0;
  const int MOD = 20;  // 1 = every loop, 2 = every other loop, mod3 = every 3rd loop, etc.
  logLoop++;
  if ((logLoop % MOD) == 0) { 
    if (isRunning && isRouteInProgress) {
      addLog(imu.gHeading, currentLoc.x, currentLoc.y, routeCurrentAction);
    }
  }
}
