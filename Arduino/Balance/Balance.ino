/*
 * This script is for the balance project 
 * to test and tweak the roll / pitch pid value 
 * refer to manual to see how to setup the rig
 * 
 * Don't forget to setup the right pin in the Config.h file
 * See the config.h file to tweak the PID values
 * 
 * 
 * Made by Nils Lahaye 2021
 */


#include "Config.h"

/******************************************/
/*            GLOBAL VARIABLES            */
/******************************************/

MPU6050 mpu (Wire);

Servo ESC_FL, ESC_FR;
float ESC_val_FL, ESC_val_FR;


float pid_i_mem_Roll, pid_setpoint_Roll, gyro_input_Roll, pid_output_Roll, pid_last_d_error_Roll; //Roll PID calcul value

double throttle=1300; //initial value of throttle to the motors

bool isReady = false;

/*******************************/
/*            CODE             */
/******************************/
void setup() {
  Serial.begin(115200);
  
  Serial.println(F("\nSend any character to begin : "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println("Seting up mpu6050"); 
  mpu.Initialize();
  mpu.SetGyroOffsets(GyroX,GyroY,GyroZ); //Seting the gyrooffset from the Config.h file
  Serial.println("");

  
  Serial.println("Seting up ESCs");

  //Set the pin for each esc
  ESC_FL.attach(EscPinFR);
  ESC_FR.attach(EscPinFL);
  Serial.print('>');

  //Set esc to there idle speed
  ESC_FL.writeMicroseconds(escIdle); 
  ESC_FR.writeMicroseconds(escIdle);
  Serial.print('>');

  for(int i; i < 10; i++) {//Delay with feedback to let the esc boot
    Serial.print('*');
    delay(500);
  }
  Serial.println("");

  
  Serial.println("Seting up Reciver");

  pinMode(ThrotlePin, INPUT);
  pinMode(RollPin, INPUT);


  isReady = true;
  Serial.println("Ready");
}

void loop() {
  if(!isReady) return;//Wait for ready

  //Get the gyro value
  mpu.Execute();
  
  
  float x = mpu.GetAngX(); //Get the x angle
  gyro_input_Roll = x; //Set the gyro angle for the PID
  
  int roll = map(pulseIn(RollPin, HIGH),rollMin,rollMax,-65,65); //Get the roll value
  pid_setpoint_Roll = roll; //Set the setpoint for the roll 

  pidCompute(); //compute PID
  
  throttle = map(pulseIn(ThrotlePin,HIGH),throttleMin,throttleMax,escMin,escMax); //Get the current throttle

  //Set the new ESC val
  ESC_val_FL = throttle + pid_output_Roll;
  ESC_val_FR = throttle - pid_output_Roll;
  
  
  
  //Make sure esc val ar in range
  if (ESC_val_FL < escMin) ESC_val_FL = escMin;
  if (ESC_val_FR < escMin) ESC_val_FR = escMin;

  if(ESC_val_FL > escMax)ESC_val_FL = escMax;
  if(ESC_val_FR > escMax)ESC_val_FR = escMax;  
  
  //Write the ESC values
  ESC_FL.writeMicroseconds(ESC_val_FL);
  ESC_FR.writeMicroseconds(ESC_val_FR);


  //Print for debuging
  Serial.print(pid_p_Roll);
  Serial.print("\t");
  Serial.print(pid_d_Roll);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(pid_output_Roll);
  Serial.print("\t");
  Serial.print(ESC_val_FL);
  Serial.print("\t");
  Serial.print(ESC_val_FR);
  Serial.print("\t");
  Serial.print(pid_setpoint_Roll);
  Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.println(throttle);
}


//Compute all of the pid values
void pidCompute()
{
    float _pid_error;

    //Roll
    _pid_error = gyro_input_Roll - pid_setpoint_Roll; 
    pid_i_mem_Roll += pid_i_Roll * _pid_error;
    if(pid_i_mem_Roll > pid_max_Roll)pid_i_mem_Roll = pid_max_Roll;
    else if(pid_i_mem_Roll < pid_max_Roll * -1)pid_i_mem_Roll = pid_max_Roll * -1;

    pid_output_Roll = pid_p_Roll * _pid_error + pid_i_mem_Roll + pid_d_Roll * (_pid_error - pid_last_d_error_Roll);
    if(pid_output_Roll > pid_max_Roll)pid_output_Roll = pid_max_Roll;
    else if(pid_output_Roll < pid_max_Roll * -1)pid_output_Roll = pid_max_Roll * -1;

    pid_last_d_error_Roll = _pid_error;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
