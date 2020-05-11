/*****************************************************************************-
 *                        Motor.ino
 *****************************************************************************/


/******************************************************************************
 *  motorInit()
 *****************************************************************************/
void motorInit() {
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(ENC_A_RIGHT_PIN, INPUT);
  pinMode(ENC_A_LEFT_PIN, INPUT);
  pinMode(ENC_B_RIGHT_PIN, INPUT);
  pinMode(ENC_B_LEFT_PIN, INPUT);

  analogWriteFrequency(PWM_RIGHT_PIN, 20000);
  analogWriteFrequency(PWM_LEFT_PIN, 20000);

  digitalWrite(DIR_RIGHT_PIN, LOW);
  digitalWrite(DIR_LEFT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT_PIN), encoderIsrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_RIGHT_PIN), encoderIsrRightB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT_PIN), encoderIsrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_LEFT_PIN), encoderIsrLeftB, CHANGE);
}



/******************************************************************************
 * encoderIsr???()
 *****************************************************************************/
void encoderIsrRightA() { encoderIsrRight(true ^ ENCODER_PHASE); }
void encoderIsrRightB() { encoderIsrRight(false ^ ENCODER_PHASE); }
void encoderIsrRight(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool oldIsFwd = true;
  boolean encA = (digitalReadFast(ENC_A_RIGHT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_RIGHT_PIN) == HIGH) ? true : false;
  bool isChangeDir = false;
  bool isFwd = true;

  if ((isA && (encA == oldA)) || (!isA && (encB == oldB))) {
    interruptErrorsRight++; // Bogus unterrupt!
    return;
  }
  oldA = encA;
  oldB = encB;
  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  
  // Get direction.
  if (isA && (encA == encB)) isFwd = false;
  else if (!isA && (encA != encB)) isFwd = false;
  else isFwd = true;
  if (oldIsFwd != isFwd) isChangeDir = true;
  oldIsFwd = isFwd;
  
  tickPositionRight = (isFwd) ? (tickPositionRight + 1) : (tickPositionRight - 1);
  if (isChangeDir) {
    changeDirRight++;
    return; // Don't time reversals
  }
  int tickPeriodRight = (int) (tickTimeRight - lastTickTime);
  if (!isFwd) tickPeriodRight = - tickPeriodRight;
//addLog((float) isFwd, (float) tickPeriodRight, 0.0, 0.0);  
  tickSumRight += tickPeriodRight;
  tickCountRight++;
} // encoderIsrRight()


/**************************************************************************.
   encoderIsrLeft()
 **************************************************************************/
void encoderIsrLeftA() { encoderIsrLeft(true ^ ENCODER_PHASE); }
void encoderIsrLeftB() { encoderIsrLeft(false ^ ENCODER_PHASE); }
void encoderIsrLeft(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool oldIsFwd = true;
  boolean encA = (digitalReadFast(ENC_A_LEFT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_LEFT_PIN) == HIGH) ? true : false;
  bool isChangeDir = false;
  bool isFwd = true;

  if ((isA && (encA == oldA)) || (!isA && (encB == oldB))) {
    interruptErrorsLeft++; // Bogus unterrupt!
    return;
  }
  oldA = encA;
  oldB = encB;
  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  
  // Get direction.
  if (isA && (encA == encB)) isFwd = true;
  else if (!isA && (encA != encB)) isFwd = true;
  else isFwd = false;
  if (oldIsFwd != isFwd) isChangeDir = true;
  oldIsFwd = isFwd;
  
  tickPositionLeft = (isFwd) ? (tickPositionLeft + 1) : (tickPositionLeft - 1);
  if (isChangeDir) {
    changeDirLeft++;
    return; // Don't time reversals
  }
  int tickPeriodLeft = (int) (tickTimeLeft - lastTickTime);
  if (!isFwd) tickPeriodLeft = - tickPeriodLeft;
  tickSumLeft += tickPeriodLeft;
  tickCountLeft++;
} // end encoderIsrLeft();



/******************************************************************************
   readSpeed????()  Called every loop from CheckMotor()
 *****************************************************************************/
void readSpeedRight() {
  noInterrupts();
  int sum = tickSumRight;
  int count =  tickCountRight;
  tickSumRight = 0L;
  tickCountRight = 0;
  interrupts();
  if (count == 0) {
    float newWKph = USEC_TO_KPH / ((float) (micros() - tickTimeRight));
    if (wKphRight < 0.0) newWKph = -newWKph;
    if (newWKph > 0.0) {
      if (newWKph < wKphRight) wKphRight = newWKph; // Set new if lower
    } else {
      if (newWKph > wKphRight) wKphRight = newWKph; // Set new if lower
    }
  } else {
    wKphRight =  (USEC_TO_KPH * ((float) count)) / ((float) sum) ;
  }
}

void readSpeedLeft() {
  noInterrupts();
  long sum = tickSumLeft;
  int count =  tickCountLeft;
  tickSumLeft = 0L;
  tickCountLeft = 0;
  interrupts();
  if (count == 0) {
    float newWKph = USEC_TO_KPH / ((float) (micros() - tickTimeLeft));
    if (wKphLeft < 0.0) newWKph = -newWKph;
    if (newWKph > 0.0) {
      if (newWKph < wKphLeft) wKphLeft = newWKph; // Set new if lower
    } else {
      if (newWKph > wKphLeft) wKphLeft = newWKph; // Set new if lower
    }
  }
  else {
    wKphLeft = (USEC_TO_KPH * ((float) count)) / ((float) sum);
  }
}

 

/******************************************************************************
   runMotors()
 *****************************************************************************/
void runMotors() {
  runMotorRight();
  runMotorLeft();
  wKph = (wKphLeft + wKphRight) / 2.0;
  tickPosition = tickPositionRight + tickPositionLeft;
  tickMeters = tickPosition / (TICKS_PER_METER * 2.0);
}



/******************************************************************************
 * runMotor????()  Called every loop
 *****************************************************************************/
void runMotorRight() {
  float motorGain = K0;
  readSpeedRight();

  float wsError = (float) (targetWKphRight - wKphRight);
  if (abs(targetWKphRight) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWKphRight) * 8.0);
  }
  motorTargetKphRight = targetWKphRight + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(motorTargetKphRight * KPH_TO_PW);            // Pw for the target.
  setMotorRight(pw, motorTargetKphRight > 0.0);
}

void runMotorLeft() {
  float motorGain = K0;
  readSpeedLeft();

  float wsError = (float) (targetWKphLeft - wKphLeft);
  if (abs(targetWKphLeft) < 0.5) {  // reduce gain below .5 fps
    motorGain = 1.0 + (abs(targetWKphLeft) * 8.0);
  }
  motorTargetKphLeft = targetWKphLeft + (wsError * motorGain);  // Target speed to correct error
  float pw = abs(motorTargetKphLeft * KPH_TO_PW);            // Pw for the target.
  setMotorLeft(pw, motorTargetKphLeft > 0.0);
}


/*******************************************************************************
   setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, (isFwd) ? LOW : HIGH);
  analogWrite(PWM_RIGHT_PIN, pw);\
  motorRightPw = pw;  // For panic
}

void setMotorLeft(int pw, bool isFwd) {
  if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_LEFT_PIN, (isFwd) ? HIGH : LOW);
  analogWrite(PWM_LEFT_PIN, pw);
  motorLeftPw = pw;
}
