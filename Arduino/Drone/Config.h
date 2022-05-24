/************************************/
/*         Included Libraries       */
/************************************/
#include <Arduino.h>
#include <TinyMPU6050.h>
#include <Servo.h>

/************************************/
/*              Pinout              */
/************************************/

//ESC                                                        CW    CCW
#define EscPinFL 11 //CW                                        ╲  ╱
#define EscPinFR 10 //CCW                                        ▉     ↑ forward
#define EscPinBL 9 //CCW                                       ╱  ╲
#define EscPinBR 6 //CW                                     CCW   CW

//Remote
#define ThrotlePin A0
#define RollPin A1
#define PitchPin A2  
#define YawPin A3
#define Extra1Pin A4
#define Extar2Pin A5

//Piezo
#define PiezoPin 3

/************************************/
/*           Variables              */
/************************************/

/******PID Values******/
extern const double pid_p_Roll=1.73, pid_i_Roll=0.05, pid_d_Roll=13.47; //PID val for roll
extern const double pid_p_Pitch=1.73, pid_i_Pitch=0.05, pid_d_Pitch=13.47; //PID val for pitch
extern const double pid_p_Yaw=3.0, pid_i_Yaw=0.04, pid_d_Yaw=0.0; //PID val for yaw

extern const int pid_max_Roll = 400; //Max PID output for roll
extern const int pid_max_Pitch = 400; //Max PID output for pitch
extern const int pid_max_Yaw = 400; //Max PID output for yaw

/******ESC Values******/
extern const int escIdle = 1000; //ESC idle speed
extern const int escMin = 1100; //ESC min working speed (for all esc)
extern const int escMax = 2000; //ESC max working spee (for all esc)

/******RC Values******/
//If axis are inversed switch the min and the max
extern const int throttleMin=996, throttleMax=1988;
extern const int rollMin=1010, rollMax=1986;
extern const int pitchMin=994, pitchMax=1983;
extern const int yawMin=995, yawMax=1978;
//RC value mapping
extern const int rollMinAngle=-65, rollMaxAngle=65;
extern const int pitchMinAngle=-45, pitchMaxAngle=45;
extern const int yawMinAngle=-10, yawMaxAngle=10;
//Yaw move speed
extern const float yawStrenght = 0.1;

/******MPU6050 Values******/
extern const float GyroX=-109.47, GyroY=51.02, GyroZ=-109.47;//Gyro offset 
