/*****************************************************************************-
 *                            SixPotatoe.ino
 *****************************************************************************/
#include <Wire.h>
#include "IMU.h"
#include "Routes.h"
#include "Defs.h"

#ifdef YELLOW_435
//const enum motorType MOTOR_TYPE = yellow435;
const float MOTOR_RPM = 435.0;        // RPM at 12V
const float MOTOR_GEAR_RATIO = 13.7;
const float MOTOR_EVENTS = 28.0;       // Encoder ticks per motor revolution
const bool ENCODER_PHASE = true;
const float K0_MULT = 0.42;
const float K5_MULT = 1.5;
const float K13_MULT = 1.0;
const float K14_MULT = 1.0;
#endif // YELLOW_435

#ifdef HD_437
//const enum motorType MOTOR_TYPE = hd437;
const float MOTOR_RPM = 437.0;          // RPM at 12V
const float MOTOR_GEAR_RATIO = 19.2;
const float MOTOR_EVENTS = 48.0;        // Encoder ticks per motor revolution
const bool ENCODER_PHASE = false;
const float K0_MULT = 0.42;
const float K5_MULT = 1.5;
const float K13_MULT = 1.0;
const float K14_MULT = 1.0;
#endif // HD_437

#ifdef HD_612
//const enum motorType MOTOR_TYPE = hd612;
const float MOTOR_RPM = 612.0;          // RPM at 12V
const float MOTOR_GEAR_RATIO = 13.7;
const float MOTOR_EVENTS = 48.0;        // Encoder ticks per motor revolution
const bool ENCODER_PHASE = false;
const float K0_MULT = 0.42;            // Motor gain
const float K5_MULT = 1.5;             // Speed error to angle
const float K13_MULT = 1.0;            // Constrain pitch error
const float K14_MULT = 1.0;            // Angle error gain
#endif // HD_612

#ifdef YELLOW_1150
// Tested good on street 20-Jun-20
//const enum motorType MOTOR_TYPE = yellow1150;
const float MOTOR_RPM = 1150.0;         // RPM at 12V
const float MOTOR_GEAR_RATIO = 5.2;
const float MOTOR_EVENTS = 28.0;        // Encoder ticks per motor revolution
const bool ENCODER_PHASE = true;
const float K0_MULT = 0.9;              // Motor gain
const float K5_MULT = 0.8;              // Speed error to angle
const float K13_MULT = 1.0;             // Constrain pitch error
const float K14_MULT = 1.0;             // Angle error gain
#endif // YELLOW_1150

const float MAX_MOTOR_RPM = (BATTERY_VOLTS / NOMINAL_VOLTS) * MOTOR_RPM;
const float TICKS_PER_ROTATION = MOTOR_EVENTS * MOTOR_GEAR_RATIO;
const float MAX_MOTOR_KPH =  (WHEEL_DIA_MM * M_PI * MAX_MOTOR_RPM * 60.0) / 1000000.0;
const float USEC_TO_KPH = (3600.0 * WHEEL_DIA_MM * M_PI) / TICKS_PER_ROTATION;
const float KPH_TO_PW = 255.0 / MAX_MOTOR_KPH;
const float TICKS_PER_METER = (1000.0 / (M_PI * WHEEL_DIA_MM)) * TICKS_PER_ROTATION;

/*****************************************************************************-
 *  Pin definitions
 *****************************************************************************/
const int PWM_LEFT_PIN    = 14;
const int PWM_RIGHT_PIN   = 15;
const int DIR_LEFT_PIN    = 16;
const int DIR_RIGHT_PIN   =  9;

const int ENC_B_LEFT_PIN  = 20;
const int ENC_A_LEFT_PIN  = 21;
const int ENC_B_RIGHT_PIN = 22;
const int ENC_A_RIGHT_PIN = 23;

const int CH1_RADIO_PIN   =  7;
const int CH2_RADIO_PIN   =  6;
const int CH3_RADIO_PIN   =  5;
const int CH4_RADIO_PIN   =  4;
const int CH5_RADIO_PIN   =  3;
const int CH6_RADIO_PIN   =  2;

const int LED_PIN         = 13;
const int LED_STAT_PIN    = 12;
const int SW_STAT_PIN     = 11;
const int BATTERY_PIN     = 17;

enum BlinkState {
  BLINK_OFF,        //  motors off, no route
  BLINK_ON,         //  motors on,  no route 
  BLINK_SLOW_FLASH, //  motors off, running route
  BLINK_SLOW,       //  motors on,  running route
  BLINK_FAST_FLASH  //  motors on,  fallen
};

BlinkState currentBlink = BLINK_OFF;

//#define N_FLOAT_LOGS 1  // max before running out of memory
#define N_FLOAT_LOGS 7000  // max before running out of memory
float logFloats[8][N_FLOAT_LOGS];
int logFloatCount = 0;
boolean isLogFloatWrap = false;
String logHeader = "No Header";
#define N_TICK_LOGS 1 // 40,000 near limit
//#define N_TICK_LOGS 20000 // 40,000 near limit
unsigned long tickLogRight[N_TICK_LOGS];
byte intLogRight[N_TICK_LOGS];
int tcRight = 0;
unsigned long tickLogLeft[N_TICK_LOGS];
byte intLogLeft[N_TICK_LOGS];
int tcLeft = 0;

// Motor varialbles
const float K0_RESULT = K0 * K0_MULT; // adjust gain by motor torque
const float K5_RESULT = K5 * K5_MULT;
const float K14_RESULT = K14 * K14_MULT;  // Error gain.
const float K13_RESULT = K13 * K13_MULT;

// Motor state bits
const byte STATE_DIR = B00001000;
const byte STATE_ISA = B00000100;
const byte STATE_A =   B00000010;
const byte STATE_B =   B00000001;

const int N_TICK_BUFF = 200;
volatile unsigned long tickTimesRight[N_TICK_BUFF];
volatile unsigned long tickTimesLeft[N_TICK_BUFF];
volatile byte tickStatesRight[N_TICK_BUFF];
volatile byte tickStatesLeft[N_TICK_BUFF];
volatile int tickPtrRight = 0;
volatile int tickPtrLeft = 0;
volatile int interruptErrorsRightA = 0;
volatile int interruptErrorsLeftA = 0;
volatile int interruptErrorsRightB = 0;
volatile int interruptErrorsLeftB = 0;
volatile int interruptErrorsRightC = 0;
volatile int interruptErrorsLeftC = 0;
volatile int ticksRight = 0;
volatile int ticksLeft = 0;

float wKphRight = 0.0;
float wKphLeft = 0.0;
float wKph = 0.0;
float targetWKphRight = 0.0;
float motorTargetKphRight = 0.0;
float targetWKphLeft = 0.0;
float motorTargetKphLeft = 0.0;
int motorPwRight = 0;
int motorPwLeft = 0;

// Run variables
float coKph = 0.0;
float balanceTargetKph = 0.0;
float balanceSteerAdjustment = 0.0;
float targetWKph = 0.0;
unsigned long timeMilliseconds = 0UL;
unsigned long timeMicroseconds = 0UL;
bool isRunning = false;
bool isBowlBalancing = false;
bool isAir = false;
bool isUpright = false;
bool isZeroG = false;
bool isBatteryWarn = false;
bool isBatteryCritical = false;
float zeroGKph = 0.0;
int tickPositionRight = 0;
int tickPositionLeft = 0;
int tickPosition = 0L;
double tickMeters = 0.0;
double coTickPosition = 0.0;

// RC variables
volatile float controllerX = 0.0;   // ch1 steering
float controllerY = 0.0;   // ch2 accelerator, limited rate
volatile boolean ch3State = false;  // ch3 toggle
volatile int ch4State = 0;          // ch4 3-position switch
volatile float ch5Val = 0.0;        // ch5 top left potentiometer
volatile int ch5State = 0;          // ch5 states 1-5
volatile float ch6Val = 0.0;        // ch6 top right potentiometer.

// Nav public variables
struct loc {
  double x;
  double y;
};
struct loc currentLoc;
struct loc targetLoc;
struct loc pivotLoc;
//struct loc hugStartLoc;
struct loc coSetLoc;
unsigned long timeRun = 0;
char routeCurrentAction = 0;
int routeStepPtr = 0;
boolean isStartReceived = false;
boolean isRouteInProgress = false;
float routeKph = 0.0;
float turnRadius = 0.0;
String *currentRoute = routeTable[0];
const float STEP_ERROR = -42.42;
int originalStepStringPtr = 0;
String stepString = "";
int numLen = 0;
double dDiff = 0.0D;
float targetBearing = 0.0;
float targetDistance = 0.0;
float targetRunDistance = 0.0;
float startTurnBearing = 0.0;
float currentRotation = 0.0;
float stepRotation = 0.0;
double stepDistance = 0.0D;
double currentDistance = 0.0D;
double startOrientation = 0.0;
struct loc startLoc;
enum {
  PHASE1,
  PHASE2,
  PHASE3
} getUpPhase = PHASE1;
unsigned long getUpStartTime = 0;
const int GU_PHASE_1 = 0;
bool isGetUpPhase1 = true;
bool isGetUpBack = false;
unsigned long getUpPhase1Ms = 400;
unsigned long getUpPhase2Ms = 200;
float getUpPhase1Kph = 6.0;
float getUpPhase2Kph = 12.0;
  
IMU imu;



/*****************************************************************************_
 * setup()
 *****************************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
 
  // Motor pins are initialized in motorInit()
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_STAT_PIN, OUTPUT);
  pinMode(SW_STAT_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);
    
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_STAT_PIN, LOW);

  rcInit(); 
  imu.imuInit(Wire, 1);
  motorInit();
  delay(100); // For switches?
}



/*******************************************************************************
 * loop()
 ******************************************************************************/
void loop() {
  if (IS_TEST1) systemTest1();
  else if (IS_TEST2)  systemTest2();
  else if (IS_TEST3)  systemTest3();
  else if (IS_TEST4)  systemTest4();
  else run();
}



/*****************************************************************************-
 * systemTest?()  
 *           TODO Explain test
 *****************************************************************************/

void systemTest1() {  // Check IMU
  static unsigned long lastT = 0;
  if (imu.isNewImuData()) {
    unsigned long newT = millis();
    Serial.printf(
            "Pitch:%6.2f   Roll:%6.2f   Yaw:%6.2f   Period:%2d ms\n", 
            imu.maPitch, imu.maRoll, imu.maYaw, ((int) (newT - lastT)));
    lastT = newT;
    blinkTeensy();
  }
}

void systemTest2() {  // Check the RC controller
  static boolean toggle = false;
  static int m = 0;
  if (imu.isNewImuData()) {
    if ((m++ % 10) == 0) { 
      digitalWrite(LED_PIN, (toggle = !toggle) ? HIGH : LOW);
      Serial.printf("ch1:%5.2f     ch2:%5.2f     ch3:%1d     ch4:%1d     ch5:%5.2f   ch5St5ate:%1d    ch6:%5.2f\n", 
                    controllerX, controllerY, ch3State, ch4State, ch5Val, ch5State, ch6Val);
    }
    blinkTeensy();
  }
}

void systemTest3() {  // Check motor controlers and encoders
  while (true) {
    commonTasks();
    if (imu.isNewImuData()) {
      int x = (int) (controllerX * 20.0);
      int y = (int) (controllerY * 255.0);
      int r = y + x;
      int l = y - x;
      setMotorRight(r);
      setMotorLeft(l);
      readSpeedRight();
      readSpeedLeft();
      Serial.printf("%7.2f %7.2f %5d %5d %5d\n", wKphRight, wKphLeft, r, l, isRunning);
      blinkTeensy();
    }
  }
}

void systemTest4() {  // Check speed control
  while (true) {
    commonTasks();
    if (imu.isNewImuData()) {
      float x = (controllerX * 5.0);
      float y = (controllerY * 10.0);
      targetWKphRight = y + x;
      targetWKphLeft = y - x;
      runMotors();
      Serial.printf("%7.2f %7.2f %7.2f %7.2f %5d\n", wKphRight, wKphLeft, x, y, isRunning);
      blinkTeensy();
    }
  }
}
