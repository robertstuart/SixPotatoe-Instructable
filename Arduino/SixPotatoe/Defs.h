/*****************************************************************************-
 *                            Defs.h
 *    Normally, this will be the only file that the user needs to modify.
 *    This is where the motor and battery are specified.  There are also
 *    a number of "K" constants that can be adjusted to tune various aspects
 *    of SixPotatoe's performance.  
 *****************************************************************************/

// The motor is defined here.  Uncomment the line for the motor you are using.
//#define YELLOW_435   
#define YELLOW_1150   
//#define HD_437
//#define HD_612
//#define HD_1621

const float WHEEL_DIA_MM = 120.6;  // The diameter of the wheel.
const float NOMINAL_VOLTS = 12.0;  // Voltage at which motor is rated.
const float BATTERY_VOLTS = 25.2;  // 6 cells at 4.2V per cell


// Tunable constants
//const float K0  = 4.0;      // Motor gain
const float K0  = 8.0;      // Motor gain
const float K1  = 1.6;
const float K2  = 0.05;     // 1.0 passes all hf, near zero passes only low freq.
const float K3 = 0.213;     // Accelerometer to Kph
const float K5  = 2.0;      // Speed error to angle
const float K8  = 0.2;      // bowl roll compensation at top.
const float K10 = 1.4;      // accelerometer pitch offset
const float K12 = 50.0;     // +- constraint on target pitch
//float K13 = 30.0;           // +- constraint on pitch error to prevent too rapid righting
const float K13 = 20.0;     // +- constraint on pitch error to prevent too rapid righting
const float K14 = 0.15;     // Angle error to Kph
const float K15 = 70;       // pitch beyond which is considered to not be upright
const int   K16 = 50;       // ms time for pitch < K16 to be not upright
const float K17 = 0.1;      // "D"
const int   K20 = 80;       // LED brightness, 0-255;
const float K21 = 0.95;     // TC for accelCoKph
//const float K30 = 22.0;     // Maximum Kph target for controller, 612 RPM
const float K30 = 16.0;     // Maximum Kph target for controller, 437 RPM
const float K31 = 5.0;      // Maximum speed for Kph on ground.
const float K32 = 2.0;      // Maximum sterring on ground.


// One of the following are set to "true" for initial testing.
const bool IS_TEST1 = false;  // Set to be true for the 1st system test.
const bool IS_TEST2 = false;  // Set to be true for the 2nd system test.
const bool IS_TEST3 = false;  // Set to be true for the 3nd system test.
const bool IS_TEST4 = false;  // Set to be true for the 4th system test.
const bool IS_TEST5 = false;  // Set to be true for the 5th system test.
