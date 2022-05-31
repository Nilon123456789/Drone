/*
 * This is the Main script of the Drone
 * Please run the setup script first to
 * Save the gyro and reciver vales.
 * Don't forget to check the pin of the
 * component and the wiring. Also it is
 * sugested that you run the balance script
 * to setup the pid values.
 * 
 * 
 * Made by Nils Lahaye 2022
 */

#include "Config.h"

/******************************************/
/*            GLOBAL VARIABLES            */
/******************************************/

MPU6050 mpu (Wire);

Servo ESC_FL, ESC_FR, ESC_BL, ESC_BR;
float ESC_val_FL = 1000, ESC_val_FR  = 1000, ESC_val_BL  = 1000, ESC_val_BR  = 1000;


float pid_i_mem_Roll, pid_setpoint_Roll, gyro_input_Roll, pid_output_Roll, pid_last_d_error_Roll; //Roll PID calcul value
float pid_i_mem_Pitch, pid_setpoint_Pitch, gyro_input_Pitch, pid_output_Pitch, pid_last_d_error_Pitch; //Pitch PID calcul value
float pid_i_mem_Yaw, pid_setpoint_Yaw, gyro_input_Yaw, pid_output_Yaw, pid_last_d_error_Yaw; //Yaw PID calcul value

bool isReady = false;

/*******************************/
/*            SETUP            */
/*******************************/

void setup() {
  Serial.begin(115200);

  tone(PiezoPin, 3000, 250);

  Serial.println("Seting up mpu6050");
  mpu.Initialize();
  mpu.SetGyroOffsets(GyroX,GyroY,GyroZ); //Seting the gyrooffset from the Config.h file
  Serial.println("");
  mpu.Calibrate();

  tone(PiezoPin, 3000, 250);
  delay(250);
  tone(PiezoPin, 3000, 250);
  
  Serial.println("Seting up ESCs");
  
  //Set the pin for each ESC
  ESC_FL.attach(EscPinFR);
  ESC_FR.attach(EscPinFL);
  ESC_BL.attach(EscPinBL);
  ESC_BR.attach(EscPinBR);
  Serial.print('>');
  
  //Set the ESC to idle speed
  ESC_FL.writeMicroseconds(escIdle); 
  ESC_FR.writeMicroseconds(escIdle);
  ESC_BL.writeMicroseconds(escIdle); 
  ESC_BR.writeMicroseconds(escIdle);
  Serial.print('>');

  for(int i; i < 15; i++) {//Delay with feedback to let the esc boot
    Serial.print('*');
    delay(1000);
  }
  Serial.println("");

  tone(PiezoPin, 3000, 750);
  
  Serial.println("Seting up Reciver");

  pinMode(ThrotlePin, INPUT);
  pinMode(RollPin, INPUT);
  pinMode(PitchPin, INPUT);
  pinMode(YawPin, INPUT);

  tone(PiezoPin, 3000, 750);
  tone(PiezoPin, 1000, 500);
  tone(PiezoPin, 3000, 750);

  isReady = true;
  Serial.println("Ready");
}


/*******************************/
/*              LOOP           */
/*******************************/

void loop() {
  if(!isReady) return;//Wait for ready
  
  //Get the new gyro value
  mpu.Execute();

  //Set the angles in an array
  float mpuAngles[3] = {mpu.GetAngX(), mpu.GetAngY(), mpu.GetAngZ()};

  //Set the new gyro Value based on the axis
  gyro_input_Roll = mpuAngles[0] * mappedAxis[0][0] + mpuAngles[1] * mappedAxis[0][1] + mpuAngles[2] * mappedAxis[0][2];
  gyro_input_Pitch = mpuAngles[0] * mappedAxis[1][0] + mpuAngles[1] * mappedAxis[1][1] + mpuAngles[2] * mappedAxis[1][2];
  gyro_input_Yaw = mpuAngles[0] * mappedAxis[2][0] + mpuAngles[1] * mappedAxis[2][1] + mpuAngles[2] * mappedAxis[2][2];

  //Get the current throttle value
  int _throttle = map(pulseIn(ThrotlePin,HIGH),throttleMin,throttleMax,escIdle,escMax);

  if(_throttle <= escMin - 50)
  {
    ESC_FL.writeMicroseconds(escIdle); 
    ESC_FR.writeMicroseconds(escIdle);
    ESC_BL.writeMicroseconds(escIdle); 
    ESC_BR.writeMicroseconds(escIdle);
    return;
  }

  //Set the new setpoint from the controller
  pid_setpoint_Roll = map(pulseIn(RollPin, HIGH),rollMin,rollMax,rollMinAngle,rollMaxAngle);
  pid_setpoint_Pitch = map(pulseIn(PitchPin, HIGH),pitchMin,pitchMax,pitchMinAngle,pitchMaxAngle);
  pid_setpoint_Yaw = pid_setpoint_Yaw + map(pulseIn(YawPin, HIGH),yawMin,yawMax,yawMinAngle,yawMaxAngle) * yawStrenght;

  //Make sure the pid_setpoint_Yaw is in the range of the MPU6050
  if (pid_setpoint_Yaw > 180) pid_setpoint_Yaw = -180 + (pid_setpoint_Yaw - 180);
  if (pid_setpoint_Yaw < -180) pid_setpoint_Yaw = 180 + (pid_setpoint_Yaw + 180);

  /*
  //Compute the pid values
  pidCompute();

  //Seting the new ESC val from the pid output
  ESC_val_FL = _throttle + pid_output_Roll + pid_output_Pitch + pid_output_Yaw;
  ESC_val_FR = _throttle - pid_output_Roll + pid_output_Pitch - pid_output_Yaw;
  ESC_val_BL = _throttle + pid_output_Roll - pid_output_Pitch - pid_output_Yaw;
  ESC_val_BR = _throttle - pid_output_Roll - pid_output_Pitch + pid_output_Yaw;
  */
  ESC_val_FL = _throttle;
  ESC_val_FR = _throttle;
  ESC_val_BL = _throttle;
  ESC_val_BR = _throttle;

  
  
  //Make sure motor val ar in range
  if (ESC_val_FL < escMin) ESC_val_FL = escMin;
  if (ESC_val_FR < escMin) ESC_val_FR = escMin;
  if (ESC_val_BL < escMin) ESC_val_BL = escMin;
  if (ESC_val_BR < escMin) ESC_val_BR = escMin;

  if(ESC_val_FL > escMax)ESC_val_FL = escMax;
  if(ESC_val_FR > escMax)ESC_val_FR = escMax;  
  if(ESC_val_BL > escMax)ESC_val_BL = escMax;
  if(ESC_val_BR > escMax)ESC_val_BR = escMax; 
  
  //Write the ESC values
  ESC_FL.writeMicroseconds(ESC_val_FL);
  ESC_FR.writeMicroseconds(ESC_val_FR);
  ESC_BL.writeMicroseconds(ESC_val_BL);
  ESC_BR.writeMicroseconds(ESC_val_BR);

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


    //Pitch
    _pid_error = gyro_input_Pitch - pid_setpoint_Pitch;
    pid_i_mem_Pitch += pid_i_Pitch * _pid_error;
    if(pid_i_mem_Pitch > pid_max_Pitch)pid_i_mem_Pitch = pid_max_Pitch;
    else if(pid_i_mem_Pitch < pid_max_Pitch * -1)pid_i_mem_Pitch = pid_max_Pitch * -1;

    pid_output_Pitch = pid_p_Pitch * _pid_error + pid_i_mem_Pitch + pid_d_Pitch * (_pid_error - pid_last_d_error_Pitch);
    if(pid_output_Pitch > pid_max_Pitch)pid_output_Pitch = pid_max_Pitch;
    else if(pid_output_Pitch < pid_max_Pitch * -1)pid_output_Pitch = pid_max_Pitch * -1;

    pid_last_d_error_Pitch = _pid_error;


    //Yaw
    _pid_error = gyro_input_Yaw - pid_setpoint_Yaw;
    pid_i_mem_Yaw += pid_i_Yaw * _pid_error;
    if(pid_i_mem_Yaw > pid_max_Yaw)pid_i_mem_Yaw = pid_max_Yaw;
    else if(pid_i_mem_Yaw < pid_max_Yaw * -1)pid_i_mem_Yaw = pid_max_Yaw * -1;

    pid_output_Yaw = pid_p_Yaw * _pid_error + pid_i_mem_Yaw + pid_d_Yaw * (_pid_error - pid_last_d_error_Yaw);
    if(pid_output_Yaw > pid_max_Yaw)pid_output_Yaw = pid_max_Yaw;
    else if(pid_output_Yaw < pid_max_Yaw * -1)pid_output_Yaw = pid_max_Yaw * -1;

    pid_last_d_error_Yaw = _pid_error;
}

//Custom map function to map with and to float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
