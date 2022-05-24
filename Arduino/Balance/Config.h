/************************************/
/*         Included Libraries       */
/************************************/
#include "Arduino.h"
#include <TinyMPU6050.h>
#include <Servo.h>

/************************************/
/*              Pinout              */
/************************************/

//ESC
#define EscPinFL 5 //CW
#define EscPinFR 9 //CCW

//Remote
#define ThrotlePin 4
#define RollPin 3

/************************************/
/*           Variables              */
/************************************/

/******PID Values******/
extern const double pid_p_Roll=1.73, pid_i_Roll=0.05, pid_d_Roll=13.47; //PID val for roll
extern const int pid_max_Roll = 400; //Max PID output for roll

/******ESC Values******/
extern const int escIdle = 1000; //ESC idle speed
extern const int escMin = 1100; //ESC min working speed (for all esc)
extern const int escMax = 2000; //ESC max working spee (for all esc)

/******RC Values******/
//If axis are inversed switch the min and the max
extern const int throttleMin=2063, throttleMax=1032;
extern const int rollMin=2063, rollMax=1032;

/******MPU6050 Values******/
extern const float GyroX=103.89, GyroY=13.66, GyroZ=103.89;//Gyro offset 
