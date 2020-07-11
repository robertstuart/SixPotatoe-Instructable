/*****************************************************************************-
 *                                 Tasks.ino
 *****************************************************************************/

 
/*****************************************************************************-
 * commonTasks()
 *****************************************************************************/
void commonTasks() {
  timeMicroseconds = micros();
  timeMilliseconds = millis();
  blinkLed();
  switches();
  checkUpright();
  checkController();
  setRunningState();
  checkConsole();
  battery();
}



/*****************************************************************************-
 * setRunningState()  Set isRunning variable to control motors
 *                    & set LED blink
 *****************************************************************************/
void setRunningState() {
  
  // Set Led flash
  if (isRouteInProgress) {
    if (!isRunning) currentBlink = BLINK_SLOW;
    else currentBlink = BLINK_SLOW_FLASH;
  } else {
    if (!isUpright && isRunning) currentBlink = BLINK_FAST_FLASH;
    else if (isUpright && isRunning) currentBlink = BLINK_ON;
    else currentBlink = BLINK_OFF;
  }
}



/*****************************************************************************-
 * checkController()  Changes in RC controller?
 *****************************************************************************/
void checkController() {
  static bool oldCh3State = false;
  static int oldCh4State = 0;

  if (oldCh3State != ch3State) { // Start/stop button
    if (ch3State == true) isRunning = true;
    else isRunning = false;
    oldCh3State = ch3State;
  }

  
  if (oldCh4State != ch4State) { // Route/GetUp button 
    if (oldCh4State == 0) {
      if (ch4State == 1) runRoute(0); // Run 1st route: GetUp
    } else if (oldCh4State == 1) {
      if (ch4State == 2) runRoute(ch5State + 1);
      if (ch4State == 0) stopRoute();
    } else { // oldCh4State == 2
      stopRoute();
    }
    
    oldCh4State = ch4State;
  }
}



/*****************************************************************************-
 * blinkLed()
 *****************************************************************************/
void blinkLed() {
  static unsigned int blinkCount;
  static unsigned long trigger = 0UL;

  if (isBatteryWarn) {
    analogWrite(LED_STAT_PIN, ((blinkCount++ / 10) % 2) ? 255 : 0);
  } else if (timeMilliseconds > trigger) {
    trigger = timeMilliseconds + 100;
    bool buState;

    blinkCount++;
    switch (currentBlink) {
      case BLINK_SLOW_FLASH:
        buState = ((blinkCount % 10) == 0);
        break;
      case BLINK_FAST_FLASH:
        buState = ((blinkCount % 2) == 0);
        break;
      case BLINK_SLOW:
        buState = (((blinkCount / 5) % 2) == 0);
        break;
      case BLINK_ON:
        buState = true;
        break;
      default:
        buState = false;
        break;
    }
    analogWrite(LED_STAT_PIN, buState ? K20 : 0);
  }
}
void blinkTeensy() {  // Just blink the Teensy. Normally IMU heartbeat.
  static unsigned int blinkCount;
  if (isBatteryWarn) {
    digitalWrite(LED_PIN, ((blinkCount++ / 10) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(LED_PIN, ((blinkCount++ / 50) % 2) ? HIGH : LOW);
  }
}



/*****************************************************************************-
 * checkUpright() Check to see if we have fallen.  Give K14 ms to get back up
 *                again before setting usUpright to false;
 *****************************************************************************/
void checkUpright() {
  static unsigned long lastUpTime = 0UL;

  boolean cState = (abs(imu.maPitch) < K15); // Current real state
  if (cState == true) {
    isUpright = true;
    lastUpTime = timeMilliseconds;
  } else {
     if (timeMilliseconds > (lastUpTime + K16)) {
        isUpright = false;
      } else {
        isUpright = true;
      }
  }
}



/*****************************************************************************-
 * checkConsole() Dump the log to the terminal so that the data can be 
 *                captured and analyzed by Excel.
 *****************************************************************************/
void checkConsole() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'd') {
      Serial.println(logHeader);
      int end = (isLogFloatWrap) ? N_FLOAT_LOGS : logFloatCount;
      for (int i = 0; i < end; i++) {
        Serial.printf("%12.3f,%9.3f,%9.2f,%9.2f,%9.2f,%9.2f,%9.2f,%9.2f\n", 
                      logFloats[0][i],
                      logFloats[1][i],
                      logFloats[2][i],
                      logFloats[3][i],
                      logFloats[4][i],
                      logFloats[5][i],
                      logFloats[6][i],
                      logFloats[7][i]);;
      }
    } else if (c == 'z') {
      Serial.println("Log set to zero.");
      logFloatCount = 0;
      isLogFloatWrap = false;
    } else if (c == 't') {
      Serial.println("--------------Right-----------");
      for (int i = 0; i < tcRight; i++) {
        Serial.printf("%d, %d\n", (int) tickLogRight[i], (int) intLogRight[i]);

      }
      Serial.println("--------------Left-----------");
      for (int i = 0; i < tcLeft; i++) {
        Serial.printf("%d, %d\n", (int) tickLogLeft[i], intLogLeft[i]);
      }
    } else if (c == 'e') {
      Serial.printf("Right Ticks: %d    ErrorA: %d   ErrorB: %d    ErrorC: %d\n",
                    ticksRight, interruptErrorsRightA, interruptErrorsRightB, interruptErrorsRightC);
      Serial.printf("Left Ticks: %d    ErrorA: %d   ErrorB: %d    ErrorC: %d\n",
                    ticksLeft, interruptErrorsLeftA, interruptErrorsLeftB, interruptErrorsLeftC);
    }
  }
}



/*****************************************************************************-
 * addTickLog???()
 *****************************************************************************/
void addTickLogRight(unsigned long time, bool isA, bool isRise) {
  tickLogRight[tcRight] = time;
  byte b = 100;
  if (isA) b += 1;
  if (isRise) b += 10;
  intLogRight[tcRight] = b;
  tcRight++;
  if (tcRight >= N_TICK_LOGS) tcRight = 0;
}
void addTickLogLeft(unsigned long time, bool isA, bool isRise) {
  tickLogLeft[tcLeft] = time;
  byte b = 100;
  if  (isA) b += 1;
  if (isRise) b += 10;
  intLogLeft[tcLeft] = b;
  tcLeft++;
  if (tcLeft >= N_TICK_LOGS) tcLeft = 0;
}





/*****************************************************************************-
 * addLog()
 *****************************************************************************/
void addLog(float a, float b, float c, float d) {
  addLog(a, b, c, d, 0.0, 0.0, 0.0, 0.0);
}
void addLog(float a, float b, float c, float d, float e, float f, float g, float h) {
  logFloats[0][logFloatCount] = a;
  logFloats[1][logFloatCount] = b;
  logFloats[2][logFloatCount] = c;
  logFloats[3][logFloatCount] = d;
  logFloats[4][logFloatCount] = e;
  logFloats[5][logFloatCount] = f;
  logFloats[6][logFloatCount] = g;
  logFloats[7][logFloatCount] = h;
  logFloatCount++;
  if (logFloatCount >= N_FLOAT_LOGS) {
    logFloatCount = 0;
    isLogFloatWrap = true;
  }
}



/*****************************************************************************-
 * switches()
 *      Check switches and debounce
 ******************************************************************************/
void switches() {
  static unsigned int timerBu = 0;
  static boolean buState = false;
  static boolean oldBuState = false;

  // Debounce blue switch 
  boolean swState = digitalRead(SW_STAT_PIN) == LOW;
  if (swState) timerBu = timeMilliseconds;
  if ((timeMilliseconds - timerBu) > 50) buState = false;
  else buState = true;
  if (buState && (!oldBuState)) {  // Blue switch press transition?
    isRunning = !isRunning;
  }
  oldBuState = buState;
}



/*****************************************************************************-
 * battery()
 *****************************************************************************/
void battery() {
  static const int VOLT_ARRAY_SIZE = 50;  // 50 seconds of measurements
  static bool isInit = false;
  static unsigned long batteryTrigger = 0UL;
  static float voltArray[VOLT_ARRAY_SIZE];
  static int voltArrayPtr = 0;
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger = timeMilliseconds + 1000;  // 1 per second
    if (!isInit) {
      isInit = true;
      for (int i = 0; i < VOLT_ARRAY_SIZE; i++) voltArray[i] = 25.0;
    }
    float battVolt = ((float) analogRead(BATTERY_PIN)) * .02603;
    if (battVolt < 5.0) { // running off of USB power?
      isBatteryCritical = isBatteryWarn = false;
    } else {
      voltArray[voltArrayPtr++] = battVolt;
      voltArrayPtr = voltArrayPtr % VOLT_ARRAY_SIZE;
      float max = 0.0;
      for (int i = 0; i < VOLT_ARRAY_SIZE; i++) {
        float v = voltArray[i];
        if (v > max) max = v;
      }
      if (max < 18.0) isBatteryCritical = true;
      if (max < 19.0) isBatteryWarn = true;
    }
  }
}



/*****************************************************************************-
 *  updateCartesian() Update the cartesian coordinates given the new
 *                    readings from the gyro.  Does not use AHRS. 
 *                    assumes measurements relative to the robot rather
 *                    than 3d space.
 *****************************************************************************/
void updateCartesian() {
  static int oldTickPosition = 0;
  static float oldGHeading = 0.0;

//  compute the Center of Oscillation Tick Position
//  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));

  // Compute the new xy position
  double dist = ((double) (tickPosition - oldTickPosition)) / (TICKS_PER_METER * 2.0);
  currentDistance += dist;
  oldTickPosition = tickPosition;
  float heading = imu.gHeading;
  currentLoc.x += sin(heading * DEG_TO_RAD) * dist;
  currentLoc.y += cos(heading * DEG_TO_RAD) * dist;
  float rotation = heading - oldGHeading;
  currentRotation += rotation;
  oldGHeading = heading;
}

void setHeading(float heading) {
  imu.gHeading = heading;
}



/**************************************************************************.
 *  rangeAngle() Set angle value between -180 and +180
 **************************************************************************/
float rangeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}



/*****************************************************************************-
 *  rcInit()
 *****************************************************************************/
void rcInit() {
  pinMode(CH1_RADIO_PIN, INPUT);
  pinMode(CH2_RADIO_PIN, INPUT);
  pinMode(CH3_RADIO_PIN, INPUT);
  pinMode(CH4_RADIO_PIN, INPUT);
  pinMode(CH5_RADIO_PIN, INPUT);
  pinMode(CH6_RADIO_PIN, INPUT);
  attachInterrupt(CH1_RADIO_PIN, ch1Isr, CHANGE);
  attachInterrupt(CH2_RADIO_PIN, ch2Isr, CHANGE);
  attachInterrupt(CH3_RADIO_PIN, ch3Isr, CHANGE);
  attachInterrupt(CH4_RADIO_PIN, ch4Isr, CHANGE);
  attachInterrupt(CH5_RADIO_PIN, ch5Isr, CHANGE);
  attachInterrupt(CH6_RADIO_PIN, ch6Isr, CHANGE);
}



/*****************************************************************************-
 *  chXIsr() Interrupt routines for radio pulses
 *****************************************************************************/
const int RC_MAX = 2150;
const int RC_MIN = 872;
const int RC_RANGE = RC_MAX - RC_MIN;
const int RC_MID = (RC_RANGE / 2) + RC_MIN ;
void ch1Isr() { // ControllerX between -1.0 and +1.0
  static unsigned long riseTime = 0UL;
  unsigned long t = micros();
  if (digitalReadFast(CH1_RADIO_PIN)) {
    riseTime = t;
  } else {
    int ch1pw = t - riseTime;
    controllerX = ( 2.0 * ((float) (ch1pw - RC_MID))) / RC_RANGE;
  }
}
void ch2Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH2_RADIO_PIN)) {
    riseTime = t;
  } else  {
    int ch2pw = t - riseTime;
    if (isBatteryWarn) controllerY = 0.0;
    else controllerY = ( 2.0 * ((float) (ch2pw - RC_MID))) / RC_RANGE;
  }
}
void ch3Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH3_RADIO_PIN)) {
    riseTime = t;
  } else {
    int ch3pw = t - riseTime;
    ch3State = (ch3pw < 1500) ? false : true;
  }
}
void ch4Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH4_RADIO_PIN)) {
    riseTime = t;
  } else {
    int ch4pw = t - riseTime;
    if (ch4pw < 1200) ch4State = 0;
    else if (ch4pw < 1800) ch4State = 1;
    else ch4State = 2;
  }
}
void ch5Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH5_RADIO_PIN)) {
    riseTime = t;
  } else {
    int ch5pw = t - riseTime;
    ch5Val = ( 2.0 * ((float) (ch5pw - RC_MID))) / RC_RANGE;
    if (ch5Val < -0.75) ch5State = 0;
    else if (ch5Val < -0.25) ch5State = 1;
    else if (ch5Val < 0.25) ch5State = 2;
    else if (ch5Val < 0.75) ch5State = 3;
    else ch5State = 4;
  }
}
void ch6Isr() {
  static unsigned long riseTime = 0UL;
  unsigned int t = micros();
  if (digitalReadFast(CH6_RADIO_PIN)) {
    riseTime = t;
  } else {
    int ch6pw = t - riseTime;
    ch6Val = ( 2.0 * ((float) (ch6pw - RC_MID))) / RC_RANGE;
  }
}
