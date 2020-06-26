/*****************************************************************************-
 *                        Motor.ino
 *****************************************************************************/
volatile unsigned long timesLeft[5];
volatile bool dirsLeft[5];
volatile int ptrLeft = 0;
volatile int tickCountLeft = 0;
volatile unsigned long timesRight[5];
volatile bool dirsRight[5];
volatile int ptrRight = 0;
volatile int tickCountRight = 0;

/*****************************************************************************-
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



/*****************************************************************************-
 * encoderIsr???()
 *****************************************************************************/
void encoderIsrRightA() { encoderIsrRight(true ^ ENCODER_PHASE); }
void encoderIsrRightB() { encoderIsrRight(false ^ ENCODER_PHASE); }
void encoderIsrRight(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
//  static bool oldIsFwd = true;
  boolean encA = (digitalReadFast(ENC_A_RIGHT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_RIGHT_PIN) == HIGH) ? true : false;
//  bool isChangeDir = false;
  bool isFwd = true;

  if ((isA && (encA == oldA)) || (!isA && (encB == oldB))) {
    interruptErrorsRight++; // Bogus unterrupt!
    return;
  }
  oldA = encA;
  oldB = encB;
//  unsigned long lastTickTime = tickTimeRight;
  tickTimeRight = micros();
  
  // Get direction.
  if (isA && (encA == encB)) isFwd = false;
  else if (!isA && (encA != encB)) isFwd = false;
  else isFwd = true;
//  if (oldIsFwd != isFwd) isChangeDir = true;
//  oldIsFwd = isFwd;
  
  tickPositionRight = (isFwd) ? (tickPositionRight + 1) : (tickPositionRight - 1);

  // New encoder balancing
  ptrRight++;
  ptrRight = ptrRight % 5;
  timesRight[ptrRight] = tickTimeRight;
  dirsRight[ptrRight] = isFwd;
  tickCountRight++;
  
//  if (isChangeDir) {
//    changeDirRight++;
//    return; // Don't time reversals
//  }
//  int tickPeriodRight = (int) (tickTimeRight - lastTickTime);
//  if (!isFwd) tickPeriodRight = - tickPeriodRight;
//  tickSumRight += tickPeriodRight;
} // encoderIsrRight()


/*****************************************************************************-
   encoderIsrLeft()
 *****************************************************************************/
void encoderIsrLeftA() { encoderIsrLeft(true ^ ENCODER_PHASE); }
void encoderIsrLeftB() { encoderIsrLeft(false ^ ENCODER_PHASE); }
void encoderIsrLeft(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
//  static bool oldIsFwd = true;
  boolean encA = (digitalReadFast(ENC_A_LEFT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_LEFT_PIN) == HIGH) ? true : false;
//  bool isChangeDir = false;
  bool isFwd = true;

  if ((isA && (encA == oldA)) || (!isA && (encB == oldB))) {
    interruptErrorsLeft++; // Bogus unterrupt!
    return;
  }
  oldA = encA;
  oldB = encB;
//  unsigned long lastTickTime = tickTimeLeft;
  tickTimeLeft = micros();
  
  // Get direction.
  if (isA && (encA == encB)) isFwd = true;
  else if (!isA && (encA != encB)) isFwd = true;
  else isFwd = false;
//  if (oldIsFwd != isFwd) isChangeDir = true;
//  oldIsFwd = isFwd;
  
  tickPositionLeft = (isFwd) ? (tickPositionLeft + 1) : (tickPositionLeft - 1);

  // New encoder balancing
  ptrLeft++;
  ptrLeft = ptrLeft % 5;
  timesLeft[ptrLeft] = tickTimeLeft;
  dirsLeft[ptrLeft] = isFwd;
  tickCountLeft++;

  
//  if (isChangeDir) {
//    changeDirLeft++;
//    return; // Don't time reversals
//  }
//  int tickPeriodLeft = (int) (tickTimeLeft - lastTickTime);
//  if (!isFwd) tickPeriodLeft = - tickPeriodLeft;
//  tickSumLeft += tickPeriodLeft;
//  tickCountLeft++;
} // end encoderIsrLeft();



/*****************************************************************************-
 * readSpeed????()  Called every loop from CheckMotor()
 *****************************************************************************/
void readSpeedRight() {
  unsigned long times[5];
  unsigned long dirs[5];
  int period = 1;
  boolean isTick;

  // Copy the arrays.
  noInterrupts();
  int ptr = (ptrRight + 1) % 5;  // Point to the oldest in the array.
  for (int i = 0; i < 5; i++) {
    times[i] = timesRight[ptr];
    dirs[i] = dirsRight[ptr];
    ptr++;
    ptr = ptr % 5;
  }
  isTick = tickCountRight > 0;
  tickCountRight = 0;
  interrupts();
  unsigned long t = micros();

  boolean d = dirs[0];
  boolean isSameDir = (d == dirs[1]) && (d == dirs[2]) && (d == dirs[3]) && (d == dirs[4]);
  boolean isAllRecent = (t - times[0]) < 50000;
  if (isAllRecent && isSameDir) {
    // Compute kph in normal running state.
    period = (times[4] - times[0]) / 4;
    wKphRight = USEC_TO_KPH / ((float) period);
    if (dirs[4] == false) wKphRight = -wKphRight;
//Serial.print('a');
  } else if (dirs[3] != dirs[4]) {
    // Change of direction.  No speed.
    wKphRight = 0.0;
//Serial.print('b');
  } else if (isTick == false) {
    // compute new kph if no tick
    float newWKph = USEC_TO_KPH / ((float) (t - times[4]));
    if (newWKph < abs(wKphRight)) {
      wKphRight = newWKph; // Set new if lower
      if (dirs[4] == false) wKphRight = -wKphRight;
    }
//Serial.print('c');
  } else {
    // compute new kph if tick
    period = times[4] - times[3];
    wKphRight = USEC_TO_KPH / ((float) period);
    if (dirs[4] == false) wKphRight = -wKphRight;
//Serial.print('d');
  }
//  if (abs(wKphRight) > 0.1) {
//    char c = (wKphRight < 0.0) ? '-' : '+';
//    int n = abs(wKphRight) * 10.0;
//    for (int i = 0; i < n; i++) {
//      Serial.print(c);
//    }
//    Serial.println();
//  }
}

void readSpeedLeft() {
  unsigned long times[5];
  unsigned long dirs[5];
  int period = 1;
  boolean isTick;

  // Copy the arrays.
  noInterrupts();
  int ptr = (ptrLeft + 1) % 5;  // Point to the oldest in the array.
  for (int i = 0; i < 5; i++) {
    times[i] = timesLeft[ptr];
    dirs[i] = dirsLeft[ptr];
    ptr++;
    ptr = ptr % 5;
  }
  isTick = tickCountLeft > 0;
  tickCountLeft = 0;
  interrupts();
  unsigned long t = micros();

  boolean d = dirs[0];
  boolean isSameDir = (d == dirs[1]) && (d == dirs[2]) && (d == dirs[3]) && (d == dirs[4]);
  boolean isAllRecent = (t - times[0]) < 50000;
  if (isAllRecent && isSameDir) {
    // Compute kph in normal running state.
    period = (times[4] - times[0]) / 4;
    wKphLeft = USEC_TO_KPH / ((float) period);
    if (dirs[4] == false) wKphLeft = -wKphLeft;
//Serial.print('a');
  } else if (dirs[3] != dirs[4]) {
    // Change of direction.  No speed.
    wKphLeft = 0.0;
//Serial.print('b');
  } else if (isTick == false) {
    // compute new kph if no tick
    float newWKph = USEC_TO_KPH / ((float) (t - times[4]));
    if (newWKph < abs(wKphLeft)) {
      wKphLeft = newWKph; // Set new if lower
      if (dirs[4] == false) wKphLeft = -wKphLeft;
    }
//Serial.print('c');
  } else {
    // compute new kph if tick
    period = times[4] - times[3];
    wKphLeft = USEC_TO_KPH / ((float) period);
    if (dirs[4] == false) wKphLeft = -wKphLeft;
//Serial.print('d');
  }
  
//  noInterrupts();
//  long sum = tickSumLeft;
//  int count =  tickCountLeft;
//  tickSumLeft = 0L;
//  tickCountLeft = 0;
//  interrupts();
//  if (count == 0) {
//    float newWKph = USEC_TO_KPH / ((float) (micros() - tickTimeLeft));
//    if (wKphLeft < 0.0) newWKph = -newWKph;
//    if (newWKph > 0.0) {
//      if (newWKph < wKphLeft) wKphLeft = newWKph; // Set new if lower
//    } else {
//      if (newWKph > wKphLeft) wKphLeft = newWKph; // Set new if lower
//    }
//  }
//  else {
//    wKphLeft = (USEC_TO_KPH * ((float) count)) / ((float) sum);
//  }

//  if (abs(wKphLeft) > 0.1) Serial.printf("%6.2f  ", wKphLeft);
//  if (abs(wKphLeft) > 0.1) {
//    char c = (wKphLeft < 0.0) ? '-' : '+';
//    int n = abs(wKphLeft) * 10.0;
//    for (int i = 0; i < n; i++) {
//      Serial.print(c);
//    }
//    Serial.println();
//  }
}



/*****************************************************************************-
 * runMotors()
 *****************************************************************************/
void runMotors() {
  runMotorRight();
  runMotorLeft();
  wKph = (wKphLeft + wKphRight) / 2.0;
  tickPosition = tickPositionRight + tickPositionLeft;
  tickMeters = tickPosition / (TICKS_PER_METER * 2.0);
}



/*****************************************************************************-
 * runMotor????()  Called every loop
 *****************************************************************************/
const float WS_TC = 0.6;  // 0 to 1;
void runMotorRight() {
  static float wsErrorLpf = 0.0;
  float g = K0_RESULT;  // motor gain computed locally
  readSpeedRight();
  
  float wsError = (float) (targetWKphRight - wKphRight);
  wsErrorLpf = (WS_TC * wsError) + ((1.0 - WS_TC) * wsErrorLpf);
  if (abs(targetWKphRight) < 1.0) {  // reduce gain below 1 kph
    g = (abs(targetWKphRight) * g);
  }
  motorTargetKphRight = targetWKphRight + (wsErrorLpf * g);  // Target speed to correct error
  float pw = abs(motorTargetKphRight * KPH_TO_PW);            // Pw for the target.
  setMotorRight(pw, motorTargetKphRight > 0.0);
}

void runMotorLeft() {
  static float wsErrorLpf = 0.0;
  float g = K0_RESULT;  // motor gain computed locally
  readSpeedLeft();

  float wsError = (float) (targetWKphLeft - wKphLeft);
  wsErrorLpf = (WS_TC * wsError) + ((1.0 - WS_TC) * wsErrorLpf);
  if (abs(targetWKphLeft) < 1.0) {  // reduce gain below 1 kph
    g = (abs(targetWKphLeft) * g);
  }
  motorTargetKphLeft = targetWKphLeft + (wsErrorLpf * g);  // Target speed to correct error
  float pw = abs(motorTargetKphLeft * KPH_TO_PW);            // Pw for the target.
  setMotorLeft(pw, motorTargetKphLeft > 0.0);
  if (isRunning) addLog(wKphLeft, wKphRight, targetWKphLeft, g);
}



/*****************************************************************************-
 * setMotor????() Set pw and diriction. pw between 0-255
 ******************************************************************************/
void setMotorRight(int pw, bool isFwd) {
  if (isBatteryCritical) pw = 0;
  else if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, (isFwd) ? LOW : HIGH);
  analogWrite(PWM_RIGHT_PIN, pw);\
  motorRightPw = pw;  // For panic
}

void setMotorLeft(int pw, bool isFwd) {
  if (isBatteryCritical) pw = 0;
  else if (!isRunning) pw = 0;
  else if (pw > 255) pw = 255;
  else if (pw < 0) pw = 0;
  digitalWrite(DIR_LEFT_PIN, (isFwd) ? HIGH : LOW);
  analogWrite(PWM_LEFT_PIN, pw);
  motorLeftPw = pw;
}
