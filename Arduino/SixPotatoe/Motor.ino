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
void encoderIsrRightA() { encoderIsrRight(true); }
void encoderIsrRightB() { encoderIsrRight(false); }
void encoderIsrRight(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool isLastA = true;
  static unsigned int lastTickTime = 0ul;
 
  boolean encA = (digitalReadFast(ENC_A_RIGHT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_RIGHT_PIN) == HIGH) ? true : false;
  unsigned long tickTime = micros();
  bool isFlipA = (encA != oldA);
  bool isFlipB = (encB != oldB);
  oldA = encA;
  oldB = encB;
//  addTickLog(tickTimeRight, isMotFwd, isA, encA, encB);
  if ((isA && !isFlipA) || (!isA && !isFlipB)) {
    interruptErrorsRightA++; // Bogus unterrupt'
    return;
  }
  if (isLastA == isA) {
    interruptErrorsRightB++;  // Bogus unterrupt or direction change.
    return;
  }
  if ((tickTime - lastTickTime) < 55) {
    interruptErrorsRightC++;  // Shouldn't happen
  }
  isLastA = isA;
  lastTickTime = tickTime;

  bool isFwd;
  if      (isA  && (encA == encB)) isFwd = true;
  else if (!isA && (encA != encB)) isFwd = true;
  else                             isFwd = false;
  byte stateByte = (isFwd) ? STATE_DIR : 0;
  stateByte     |= (isA)   ? STATE_ISA : 0;
  stateByte     |= (encA)  ? STATE_A : 0;
  stateByte     |= (encB)  ? STATE_B : 0;
  
  tickPtrRight++;
  tickPtrRight %= N_TICK_BUFF;
  tickTimesRight[tickPtrRight] = tickTime;
  tickStatesRight[tickPtrRight] = stateByte;
  ticksRight++;
} // encoderIsrRight()


/*****************************************************************************-
   encoderIsrLeft()
 *****************************************************************************/
void encoderIsrLeftA() { encoderIsrLeft(true); }
void encoderIsrLeftB() { encoderIsrLeft(false); }
void encoderIsrLeft(bool isA) {
  static bool oldA = true;
  static bool oldB = true;
  static bool isLastA = true;
  static unsigned int lastTickTime = 0ul;
 
  boolean encA = (digitalReadFast(ENC_A_LEFT_PIN) == HIGH) ? true : false;
  boolean encB = (digitalReadFast(ENC_B_LEFT_PIN) == HIGH) ? true : false;
  unsigned long tickTime = micros();
  bool isFlipA = encA == oldA;
  bool isFlipB = encB == oldB;
  oldA = encA;
  oldB = encB;
  
  if ((isA && isFlipA) || (!isA &&isFlipB)) {
    interruptErrorsLeftA++; // Bogus unterrupt'
    return;
  }
  if (isLastA == isA) {
    interruptErrorsLeftB++;  // Bogus unterrupt or direction change.
    return;
  }
  if ((tickTime - lastTickTime) < 55) {
    interruptErrorsLeftC++;  // Shouldn't happen
  }
  isLastA = isA;
  lastTickTime = tickTime;

  bool isFwd;
  if      (isA  && (encA == encB)) isFwd = false;
  else if (!isA && (encA != encB)) isFwd = false;
  else                             isFwd = true;
  byte stateByte = (isFwd) ? STATE_DIR : 0;
  stateByte     |= (isA)   ? STATE_ISA : 0;
  stateByte     |= (encA)  ? STATE_A : 0;
  stateByte     |= (encB)  ? STATE_B : 0;
  
  tickPtrLeft++;
  tickPtrLeft %= N_TICK_BUFF;
  tickTimesLeft[tickPtrLeft] = tickTime;
  tickStatesLeft[tickPtrLeft] = stateByte;
  ticksLeft++;
} // end encoderIsrLeft();



/*****************************************************************************-
 * readSpeed????()  Called every loop from CheckMotor()
 *****************************************************************************/
void readSpeedRight() {
  static float predictedKphRight = 0.0;
  static int oldTickPtr = 0;
  
  int tickSum = 0;
  int tickCount = 0;
  bool isEndFwd = false;

  // Calculate predicted kph
  float tgtKph =((float) motorPwRight) / KPH_TO_PW;
  float pdiff = predictedKphRight - tgtKph;
  predictedKphRight -= pdiff * K6;

  bool isStartFwd = ((tickStatesRight[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
  tickCount = ((tickPtrRight + N_TICK_BUFF) - oldTickPtr) % N_TICK_BUFF;
  if (tickCount == 0) { // No interrupts?
    unsigned long t = micros();
    int period = t - tickTimesRight[tickPtrRight];
    float newKph = USEC_TO_KPH / ((float) period);
    if (abs(newKph) < abs(wKphRight)) {
      wKphRight = newKph;
      if (!isStartFwd) wKphRight *= -1.0;
    }
  } else {
    if ((MOTOR_EVENTS < 30) && (tickCount < 3)) {
      oldTickPtr = tickPtrRight - 4;
      if (oldTickPtr < 0) oldTickPtr += N_TICK_BUFF;
      tickCount = 4;
    }
    for (int i = 0; i < tickCount; i++) {
      tickSum += tickTimesRight[(oldTickPtr + 1) % N_TICK_BUFF] - tickTimesRight[oldTickPtr];
      oldTickPtr++;
      oldTickPtr %= N_TICK_BUFF;
      isEndFwd = ((tickStatesRight[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
      tickPositionRight +=  (isEndFwd) ? 1 : -1;
    }
    int period = tickSum / tickCount;
    wKphRight = USEC_TO_KPH / ((float) period);
    if (isStartFwd != isEndFwd) wKphRight = 0.0;
    if (!isEndFwd) wKphRight *= -1.0;
  }
if (abs(wKphRight) > 0.1) Serial.printf("%6.2f  %6.2f %6.2f\n", wKphRight, predictedKphRight, pdiff);
if (isRunning) add8Log(wKphRight, predictedKphRight, imu.maPitch, motorPwRight, 
                       (float) interruptErrorsRightA,
                       (float) interruptErrorsRightB,
                       (float) interruptErrorsRightC,
                       0.0);

}

void readSpeedLeft() {
  static float predictedKphLeft = 0.0;
  static int oldTickPtr = 0;
  
  int tickSum = 0;
  int tickCount = 0;
  bool isEndFwd = false;

  // Calculate predicted kph
  float tgtKph =((float) motorPwLeft) / KPH_TO_PW;
  float pdiff = predictedKphLeft - tgtKph;
  predictedKphLeft -= pdiff * K6;
  
  bool isStartFwd = ((tickStatesLeft[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
  tickCount = ((tickPtrLeft + N_TICK_BUFF) - oldTickPtr) % N_TICK_BUFF;

  if (tickCount == 0) { // No interrupts?
    unsigned long t = micros();
    int period = t - tickTimesLeft[tickPtrLeft];
    float newKph = USEC_TO_KPH / ((float) period);
    if (abs(newKph) < abs(wKphLeft)) {
      wKphLeft = newKph;
      if (!isStartFwd) wKphLeft *= -1.0;
    }
  } else {
    if ((MOTOR_EVENTS < 30) && (tickCount < 3)) {
      oldTickPtr = tickPtrLeft - 4;
      if (oldTickPtr < 0) oldTickPtr += N_TICK_BUFF;
      tickCount = 4;
    }
    for (int i = 0; i < tickCount; i++) {
      tickSum += tickTimesLeft[(oldTickPtr + 1) % N_TICK_BUFF] - tickTimesLeft[oldTickPtr];
      oldTickPtr++;
      oldTickPtr %= N_TICK_BUFF;
      isEndFwd = ((tickStatesLeft[oldTickPtr] & STATE_DIR) != 0) ^ ENCODER_PHASE;
      tickPositionLeft +=  (isEndFwd) ? 1 : -1;
    }
    int period = tickSum / tickCount;
    wKphLeft = USEC_TO_KPH / ((float) period);
    if (isStartFwd != isEndFwd) wKphLeft = 0.0;
    if (!isEndFwd) wKphLeft *= -1.0;
  }
//if (abs(wKphLeft) > 0.1) Serial.printf("%6.2f  %6.2f %6.2f\n", wKphLeft, predictedKphLeft, pdiff);
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
  int pw = (int) (motorTargetKphRight * KPH_TO_PW);            // Pw for the target.
  setMotorRight(pw);
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
  int pw = (int) (motorTargetKphLeft * KPH_TO_PW);            // Pw for the target.
  setMotorLeft(pw);
}



/*****************************************************************************-
 * setMotor????() Set pw and diriction. pw between -254 & +255
 ******************************************************************************/
void setMotorRight(int pw) {
  pw = constrain(pw, -254, 255);
  motorPwRight = pw; 
  int dir = (pw >= 0) ? LOW : HIGH;
  pw = abs(pw);
  if (isBatteryCritical) pw = 0;
  if (!isRunning) pw = 0;
  digitalWrite(DIR_RIGHT_PIN, dir);
  analogWrite(PWM_RIGHT_PIN, pw);
}

void setMotorLeft(int pw) {
  pw = constrain(pw, -254, 255);
  motorPwLeft = pw;
  int dir = (pw >= 0) ? HIGH : LOW;
  pw = abs(pw);
  if (isBatteryCritical) pw = 0;
  if (!isRunning) pw = 0;
  digitalWrite(DIR_LEFT_PIN, dir);
  analogWrite(PWM_LEFT_PIN, pw);
}
