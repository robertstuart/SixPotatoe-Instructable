/*****************************************************************************-
 *                        Nav.ino
 *                   Navigate routes.
 *****************************************************************************/



/*****************************************************************************-
 *  routeControl() called every loop (200/sec).
 *          This is called as the last step in aTp7() for steering.
 *            1. Check if target reached.
 *            2. Set currentLoc.
 *            3. Adjust steering, (balanceSteerAdjustment or groundSteerAdjustment)
 *            4. Adjust speed, (balanceTargetKph, or groundKph)
 *****************************************************************************/
void routeControl() {
  boolean isNewRouteStep = false;
  
  switch (routeCurrentAction) {  

    case 'F':  // Fini
      isNewRouteStep = true;
      break;

    case 'G':  // Go
      if (currentDistance > stepDistance) isNewRouteStep = true;
      else steerTarget();
      break;

    case 'P':  // Position & wait
      if (isRunning) {
        setHeading(rangeAngle(startOrientation));
        currentLoc = startLoc;
        isNewRouteStep = true; // Wait for isRunning.
      }
      break;
      
    case 'T': // Turn
      if (abs(currentRotation) > abs(stepRotation)) isNewRouteStep = true;
      else turn();
      break;

    case 'U':  // Get up
      if (getUp()) isNewRouteStep = true;
      break;

    default:
      isRouteInProgress = false;
      Serial.printf("Illegal step: %d\n", routeStepPtr);
      break;
  } // end switch()

  if (isNewRouteStep) { // Move to next action in list.
    interpretRouteLine(getNextStepString());
    Serial.printf("Step %d: %c\n", routeStepPtr, routeCurrentAction);
  }
}



/*****************************************************************************-
 *  interpretRouteLine()
 *      Called every time the end criterion for a route step is reached.
 *      Read the new route step and set the values.
 *****************************************************************************/
boolean interpretRouteLine(String ss) {
  float floatVal = 0.0;
  stepString = ss;
  originalStepStringPtr = 0;
  Serial.print(stepString);  Serial.print(":   ");
  routeCurrentAction = stepString.charAt(0);
  stepString = stepString.substring(1);
  originalStepStringPtr++;
  isBowlBalancing = isAir = false;

  switch (routeCurrentAction) {

    case 'F':  // Fini
      stopRoute();
      break;

    case 'G':  // Go to the next waypoint
      targetLoc = readLoc();
      Serial.printf("%.2f,%.2f  ", targetLoc.x, targetLoc.y);
      if (targetLoc.y == STEP_ERROR) return false;
      floatVal =  readNum();
      Serial.printf("%.2f  ", floatVal);
      if (floatVal != STEP_ERROR) routeKph = floatVal;
      setTarget();
      stepDistance = targetDistance;
      currentDistance = 0.0;
      break;

    case 'P': // Position. Waits for runReady. Required at start
      startLoc = readLoc();
      Serial.printf("%.2f,%.2f  ", startLoc.x, startLoc.y);
      if (startLoc.y == STEP_ERROR) return false;
      startOrientation = readNum();
      Serial.printf("%.2f  ", startOrientation);
      if ((startOrientation < -180.0D) || (startOrientation > 180.0)) return false;
      floatVal = readNum();
      Serial.printf("%.2f  ", floatVal);
      if (floatVal != STEP_ERROR) routeKph = floatVal;
      else routeKph = 0.0;
   break;
      
    case 'T':  // Turn at radius ending at waypoint.
      stepRotation = readNum();
      Serial.printf("%.2f  ", stepRotation);
      if (stepRotation == STEP_ERROR) return false;
      turnRadius = readNum();
      Serial.printf("%.2f  ", turnRadius);
      if (turnRadius == STEP_ERROR) return false;
      floatVal = readNum();
      Serial.printf("%.2f  ", floatVal);
      if (floatVal != STEP_ERROR) routeKph = floatVal;
      currentRotation = 0.0;
      startTurnBearing = imu.gHeading;
      break;

    case 'U':
      if (isUpright) stopRoute();
      else setGetUp();
      break;

    default:
      Serial.println("Step error: Illegal command.");
      return false;
  }
  Serial.println();
  return true;
}



//#define RAD_TURN 10.0
#define RAD_TURN 4.0
#define END_STEER 0.5
/*****************************************************************************-
 *  steerTarget() Find the correct heading to the target and adjust the
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 *****************************************************************************/
void steerTarget() {
  setTarget();
  balanceTargetKph = routeKph;
  if ((stepDistance - currentDistance) < 0.3 ) { 
    balanceSteerAdjustment = 0.0;
  } else {
    double aDiff = rangeAngle(targetBearing - imu.gHeading);
    double d = (aDiff > 0.0) ? 1.0 : -1.0;

    balanceSteerAdjustment = (wKph / RAD_TURN) * 0.64 * d;

    // Reduce adjustment proportionally if less than X degrees.
    if (abs(aDiff) < 5.0) {
      balanceSteerAdjustment = (abs(aDiff) / 5.0) * balanceSteerAdjustment;
    }

    // Reduce speed adjustment as speed increases
    if (wKph > 11.0) balanceSteerAdjustment *= 0.3;
    else if (wKph > 8.0) balanceSteerAdjustment *= 0.5;
    else if (wKph > 5.0) balanceSteerAdjustment *= 0.7;
  }
}



/*****************************************************************************-
 *  turn() Turn with a given radius.
 *****************************************************************************/
void turn() {
  balanceTargetKph = routeKph;
  currentRotation = startTurnBearing - imu.gHeading;
  float d = (stepRotation > 0.0) ? 1.0 : -1.0;
  balanceSteerAdjustment = (wKph / turnRadius) * 0.15 * d;
}



/*****************************************************************************-
 *  setGetUp() 
 *****************************************************************************/
void setGetUp() {
  getUpPhase = PHASE1;
  isGetUpBack = (imu.maPitch > 0.0) ? true : false;
  getUpStartTime = timeMilliseconds;
}



/*****************************************************************************-
 *  getUp() 
 *****************************************************************************/
bool getUp() {
  bool ret = false;
  float tPortion = 0.0;;
  float kph = 0.0;
  unsigned long phaseTime;
  if (getUpPhase == PHASE1) { // Phase 1
    phaseTime = timeMilliseconds - getUpStartTime;
    tPortion = ((float) phaseTime) / ((float) getUpPhase1Ms);
    if (tPortion > 1.0) getUpPhase = PHASE2;
    kph = tPortion * getUpPhase1Kph;
  } else if (getUpPhase == PHASE2) {
    phaseTime = timeMilliseconds - getUpPhase1Ms - getUpStartTime;
    tPortion = ((float) phaseTime) / ((float) getUpPhase2Ms);    
    if (tPortion > 1.0) getUpPhase = PHASE3;
    kph = (getUpPhase1Kph * (1.0 - tPortion)) - (tPortion * getUpPhase2Kph);
  } else {
    ret = true;
  }
  if (!isGetUpBack) kph *= -1.0;
  targetWKphRight = targetWKphLeft = kph;
  if (isUpright) ret = true;
  return ret;
}


/*****************************************************************************-
 *  setTarget() Set the new targetBearing and targetDistance from
 *              the currentLoc.
 *****************************************************************************/
void setTarget() {
  double x =  targetLoc.x - currentLoc.x;
  double y = targetLoc.y - currentLoc.y;
  targetBearing = atan2(x, y) * RAD_TO_DEG;
  double xTargetDist = currentLoc.x - targetLoc.x;
  double yTargetDist = currentLoc.y - targetLoc.y;
  targetDistance = sqrt((xTargetDist * xTargetDist) + (yTargetDist * yTargetDist));
}



String getNextStepString() {
  return currentRoute[routeStepPtr++];
}



/*****************************************************************************-
 *  runRoute() 
 *****************************************************************************/
void runRoute(int routeNum) {
  currentRoute = routeTable[routeNum];
  routeStepPtr = 0;
  isRouteInProgress = true;
  // Run through it to see if it compiles
  while (true) {
    if (!interpretRouteLine(getNextStepString())) {
      isRouteInProgress = false;
      Serial.printf("Error step %d!\n", routeStepPtr);
      return;
    }
    if ((routeStepPtr == 1) && (routeCurrentAction != 'P')) {
      Serial.println("P required for first step");
      return;
    }
    if (!isRouteInProgress) break; 
  }
  // It made it here.  Therefore run it.
  routeStepPtr = 0;
  interpretRouteLine(getNextStepString()); // Load the first line.
  isRouteInProgress = true;
  setHeading(0.0D);
  currentLoc.x = 0.0D;
  currentLoc.y = 0.0D;
  coSetLoc = currentLoc;
}



/*****************************************************************************-
 *  stopRoute()
 *****************************************************************************/
void stopRoute() {
  isRouteInProgress = false;
  isBowlBalancing = false;
  setHeading(0.0);
}

/*****************************************************************************-
 *  Script parsing routines
 *****************************************************************************/
double readNum() {
  stripWhite();
  float num = stepString.toFloat();
  stripNum();
  if (numLen == 0) return STEP_ERROR;
  return num;
}

struct loc readLoc() {
  struct loc locLoc = {STEP_ERROR, STEP_ERROR};
  stripWhite();
  locLoc.x = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  if (stepString.charAt(0) != ',') return locLoc;
  stepString = stepString.substring(1);
  //  stripWhite();
  double y = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  locLoc.y = y;
  return locLoc;
}

void stripWhite() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c != ' ') && (c != '\t')) break;
    ptr++;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
}


void stripNum() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c == '.') || (c == '-') || ((c >= '0') && (c <= '9'))) ptr++;
    else break;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
  numLen = ptr;
}

char readChar() {
  stripWhite();
  if (stepString.length() == 0) return 0;
  char c = stepString.charAt(0);
  stepString = stepString.substring(1);
  return c;
}
