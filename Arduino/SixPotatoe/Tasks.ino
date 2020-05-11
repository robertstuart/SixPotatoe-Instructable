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
  checkLogDump();
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

  
  if (oldCh4State != ch4State) { // Route button 
    if (ch4State == 1) runRoute(ch5State);
    else if (ch4State == 0) stopRoute();
    oldCh4State = ch4State;
  }
}



/*****************************************************************************-
 * blinkLed()
 *****************************************************************************/
void blinkLed() {
  static unsigned long trigger = 0UL;
  static unsigned int blinkCount = 0;

  if (timeMilliseconds > trigger) {
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
    analogWrite(LED_BU_PIN, buState ? K20 : 0);
  }
}
void blinkTeensy() {  // Just blink the Teensy. Normally IMU heartbeat.
  static unsigned int blinkCount;
  digitalWrite(LED_PIN, ((blinkCount++ / 50) % 2) ? HIGH : LOW);
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





//
//  
////  isUpright = true; return;
//  static unsigned long tTime = 0UL; // time of last state change
//  static boolean tState = false;  // Timed state. true = upright
//
//  boolean cState = (abs(imu.maPitch) < K15); // Current real state
//  if (!cState && tState) {
//    tTime = timeMilliseconds; // Start the timer for a state change to fallen.
//  } else if (!cState) {
//    if ((timeMilliseconds - tTime) > 50) {
//      isUpright = false;
//    }
//  } else {
//    isUpright = true;
//  }
//  tState = cState;
//}



/*****************************************************************************-
 * checkLogDump() Dump the log to the terminal so that the data can be 
 *                captured and analyzed by Excel.
 *****************************************************************************/
void checkLogDump() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'd') {
      int end = (isLogStrWrap) ? N_STR_LOGS : logStrCount;
      for (int i = 0; i < end; i++) {
        Serial.println(logStrs[i]);
      }
      Serial.println(logHeader);
      end = (isLogFloatWrap) ? N_FLOAT_LOGS : logFloatCount;
      for (int i = 0; i < end; i++) {
        Serial.printf("%12.3f,%9.3f,%9.3f,%9.2f\n", 
                      logFloats[0][i],
                      logFloats[1][i],
                      logFloats[2][i],
                      logFloats[3][i]);
      }
    } else if (c == 'z') {
      Serial.println("Log set to zero.");
      logFloatCount = 0;
      isLogFloatWrap = false;
    }
  }
}



/*****************************************************************************-
 * addLog()
 *****************************************************************************/
void addLog(float a, float b, float c, float d) {
  logFloats[0][logFloatCount] = a;
  logFloats[1][logFloatCount] = b;
  logFloats[2][logFloatCount] = c;
  logFloats[3][logFloatCount] = d;
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
  boolean swState = digitalRead(SW_BU_PIN) == LOW;
  if (swState) timerBu = timeMilliseconds;
  if ((timeMilliseconds - timerBu) > 50) buState = false;
  else buState = true;
  if (buState && (!oldBuState)) {  // Blue switch press transition?
    isRunning = !isRunning;
  }
  oldBuState = buState;
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
void ch1Isr() {
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
    controllerY = ( 2.0 * ((float) (ch2pw - RC_MID))) / RC_RANGE;
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
