/*****************************************************************************-
 *                        Nav.ino
 *                   Navigate routes.
 *****************************************************************************/
unsigned long getUpPhase1Time = 100;
unsigned long getUpPhase2Time = 100;

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
  timeRun = timeMilliseconds - timeStart;
  isBowlBalancing = isAir = false;
//  targetWKphRight = targetWKphLeft = targetWKph;

  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    
    case 'B':
      if (doBowl()) isNewRouteStep = true;
      if ((timeMilliseconds - bowlStartTime) > 1000) isNewRouteStep = true;
      break;
      
    case 'F':
    case 'Q':
      isNewRouteStep = true;
      break;

    case 'A':
      if (doAir()) isNewRouteStep = true;
      break;
      
    case 'G':
      if (currentDistance > stepDistance) isNewRouteStep = true;
      else steerTarget();
      break;

    case 'P':
      if (isRunning) {
        setHeading(rangeAngle(startOrientation));
        currentLoc = startLoc;
        timeStart = timeMilliseconds;
        routeKph = 0.0;
        isNewRouteStep = true; // Wait for isRunning.
      }
      break;
      
    case 'T': // Turn
      if (abs(currentRotation) > abs(stepRotation)) isNewRouteStep = true;
      else turn();
      break;
    case 'U': // Get up from lying down.
      if (isGettingUp) {
        if (getUp(false)) isNewRouteStep = true;
      } else {
        if (isRunning) {
          getUp(true);
          isGettingUp = true;
        }
      }
      break;

    default:
      isRouteInProgress = false;
      Serial.printf("Illegal step: %d\n", routeStepPtr);
      break;
  } // end switch()

  // Move to new action if current action is done.
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
  stepString = ss;
  originalStepStringPtr = 0;
  Serial.print(stepString);  Serial.print(":   ");
  routeCurrentAction = stepString.charAt(0);
  stepString = stepString.substring(1);
  originalStepStringPtr++;
  isBowlBalancing = isAir = false;

//  (*currentValSet).v = 2.0;  // restore speed correction.

  switch (routeCurrentAction) {

    case 'A':  // Air, between "R" bowls
      airWKph = bowlWKph = wKph;
      airStartTime = timeMilliseconds;
  addLog(0.0, 0.0, 0.0, 0.0);
      break;

    case 'B': // Bowl.
      readBowl();
      if (bowlArraySize < 10) return false;
      Serial.printf("%.2f:    ", stepDistance);
      for (int i = 0; i < bowlArraySize; i++) Serial.printf("%.0f ", bowlArray[i]);
      bowlStartTime = timeMilliseconds;
  addLog(0.0, 0.0, 0.0, 0.0);
      break;
      
    case 'F':  // Fini
      stopRoute();
      break;

    case 'G':  // Go to the next waypoint
      targetLoc = readLoc();
      Serial.printf("%.2f,%.2f  ", targetLoc.x, targetLoc.y);
      if (targetLoc.y == STEP_ERROR) return false;
      routeKph =  readNum();
      Serial.printf("%.2f  ", routeKph);
      if (routeKph == STEP_ERROR) return false;
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
      break;

    case 'Q': // Quit.  Fini. Turn motors off
      isRunning = false;
      stopRoute();
      break;
      
    case 'T':  // Turn at radius ending at waypoint.
      stepRotation = readNum();
      Serial.printf("%.2f  ", stepRotation);
      if (stepRotation == STEP_ERROR) return false;
      turnRadius = readNum();
      Serial.printf("%.2f  ", turnRadius);
      if (turnRadius == STEP_ERROR) return false;
      routeKph = readNum();
      Serial.printf("%.2f  ", routeKph);
      if (routeKph == STEP_ERROR) return false;
      currentRotation = 0.0;
      startTurnBearing = imu.gHeading;
      break;


    case 'U':  // Get up from lying down
      getUpTime = readNum();
      Serial.printf("%.2f  ", getUpTime);
      if (getUpTime == STEP_ERROR) return false;
      getupSpeed = readNum();
      Serial.printf("%.2f  ", getupSpeed);
      if (getupSpeed == STEP_ERROR) return false;
      isGettingUp = false; // Wait for isRunning before starting
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

//  if (isRightTurn) {
//    xDist = (currentLoc.x - pivotLoc.x);
//    yDist = (currentLoc.y - pivotLoc.y);
//    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
//    targetTurnHeading = rangeAngle(radiusAngle + 90.0);
//    headingError = rangeAngle(imu.gHeading - targetTurnHeading);
//  } else {
//    xDist = (currentLoc.x - pivotLoc.x);
//    yDist = (currentLoc.y - pivotLoc.y);
//    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
//    targetTurnHeading = rangeAngle(radiusAngle - 90.0);
//    headingError = rangeAngle(imu.gHeading - targetTurnHeading);
//  }
//
//  float radiusError = sqrt((xDist * xDist) + (yDist * yDist)) - turnRadius;
//  float radiusAdjustment = radiusError * 0.3 * d;
//  float headingAdjustment = -headingError * 0.03;
//   speedAdjustment = radiusDiff + headingAdjustment + radiusAdjustment;
}



/*****************************************************************************-
 *  get up()  Return true if done getting up.
 *****************************************************************************/
bool getUp(bool reset) {
  const int msPhase1 = 200;
  const float speedPhase1 = 3.0;
  const int msPhase2 = 200;
  const float speedPhase2 = 4.0;
  static bool isPhase1 = true;
  static unsigned long startTime = 0UL;
  
  if (reset) {
    isPhase1 = true;
    startTime = timeMilliseconds;
  } else {
    float d = (imu.gaPitch > 0.0) ? 1.0 : -1.0;
    if (isPhase1) {
      float pct = ((float) (timeMilliseconds - startTime)) / ((float) msPhase1);
      targetWKphRight = targetWKphLeft = d * pct * speedPhase1;
      if ((timeMilliseconds - startTime) >= msPhase1) {
        isPhase1 = false;
        startTime = timeMilliseconds;
      }
    } else {
      targetWKphRight = targetWKphLeft = -d * speedPhase2;
      if ((timeMilliseconds - startTime) >= msPhase2) return true;
    }
//log(imu.maPitch, isPhase1, timeMilliseconds - startTime, targetWKphRight);
  }
  return (isUpright) ? true : false;
}



/*****************************************************************************-
 *  Bowl Compute the pitch angle for each point.
 *****************************************************************************/
boolean doBowl() {
  if (stepDistance > 0.0) {
    if (currentDistance > stepDistance) return true;
  } else {
    if (currentDistance < stepDistance) return true;
  }
  isBowlBalancing = true;
  float valA, valB;
  int ptr = abs((int) (currentDistance * 10.0));
  if (ptr < 0) {
    bowlTargetPitch = bowlArray[0];
  } else if ((ptr + 2) > bowlArraySize) {
    bowlTargetPitch = bowlArray[bowlArraySize -1];
  } else {   
    float pct = (10.0 * currentDistance) - ((float) ptr);
    if ((((int) currentDistance) + 2) >= bowlArraySize) {
      valA = valB = bowlArray[bowlArraySize - 1];
    } else {
      valA = bowlArray[ptr];
      valB = bowlArray[ptr + 1];
    }

    // Interpolate
    float diff = valB - valA;
    float valC = pct * diff;
    bowlTargetPitch = valA + valC;
  }

  // Get bowl completed which is 0.0 at beginning 1.0 at end;
  bowlCompleted = currentDistance / stepDistance;
  bowlCompleted = constrain(bowlCompleted, 0.0, 1.0);
addLog(imu.accelZ, wKph, currentDistance, imu.maPitch);
  return false;
}



/*****************************************************************************-
 *  doAir() 
 *          accelerates at 35.28 kph/sec due to gravity
 *          35.28 / 200 = 0.176 speed change per cycle
 *          830, 910, +0.5 for 17Kph
 *****************************************************************************/
bool doAir() {
  isAir = true;
  unsigned long airTime = timeMilliseconds - airStartTime;

  if (airTime < 500) {
    targetWKphRight = targetWKphLeft = bowlWKph;
  } else {
    targetWKphRight = targetWKphLeft = -(bowlWKph + 3.5);
  }
addLog(imu.accelZ, wKph, airTime, imu.maPitch);
  return (airTime > 700) ? true : false;
}



/*****************************************************************************-
 *  isTargetReached()  Return true if within 1 ft of target and is
 *                     moving away from the target.
 *                     Return true if at end of decel.
 *****************************************************************************/
//boolean isTargetReached() {
////  const int RETREAT_TIMES = 10;
////  const int RETREAT_DISTANCE = 2.0;
////  static double lastTargetDist = 10000.0D;
////  static int timesReached = 0;  if (isDecelActive && isDecelPhase) {
////    if (wKph <= 0.8) {
////      isDecelActive = isDecelPhase = false;
////      return true;
////    }
////  } else {
////    setTarget();
////    if (targetDistance < RETREAT_DISTANCE) {
////      boolean isCloser = ((lastTargetDist - targetDistance) >= 0.0D);
////      lastTargetDist = targetDistance;
////      if (isCloser) { // Getting closer?
////        timesReached = 0;
////      } else {
////        timesReached++;
////        // Return true after Nth time.
////        if (timesReached > RETREAT_TIMES) {
////          timesReached = 0;
////          return true;
////        }
////      }
////    } else {
////      timesReached = 0;
////    }
////  }
////  return false;
//}



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
  timeStart = timeMilliseconds;
  timeRun = 0;
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

void readBowl() {
  bowlArraySize = 0;
  stripWhite();
  stepDistance = stepString.toFloat();
  currentDistance = 0.0;
  stripNum();
  if (numLen == 0) return;
  while (true) {
    stripWhite();
    float num = stepString.toFloat();
    stripNum();
    if (numLen == 0) break;
    bowlArray[bowlArraySize] = num;
    bowlArraySize++;
  }
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
