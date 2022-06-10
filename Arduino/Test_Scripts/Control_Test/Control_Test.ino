
#include <TinyMPU6050.h>

//Remote
#define ThrotlePin A0
#define RollPin A1
#define PitchPin A2  
#define YawPin A3
#define Extra1Pin A4
#define Extar2Pin A5

/******ESC Values******/
extern const int escIdle = 1000; //ESC idle speed
extern const int escMin = 1100; //ESC min working speed (for all esc)
extern const int escMax = 2000; //ESC max working spee (for all esc)

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

extern const int mappedAxis[3][3] = //Place a 1 or -1 (to invert) to link the mpu input (on the top) to the axis output (on the side), one 1 max per row and colums
{
//  X , Y , Z
  {'0','1','0'}, //Pitch
  {'1','0','0'}, //Roll
  {'0','0','1'} //Yaw
};

/******MPU6050 Values******/
extern const float GyroX=-109.47, GyroY=51.02, GyroZ=-109.47;//Gyro offset 
/******PID Values******/
extern const double pid_p_Roll=1.73, pid_i_Roll=0.05, pid_d_Roll=13.47; //PID val for roll
extern const double pid_p_Pitch=1.73, pid_i_Pitch=0.05, pid_d_Pitch=13.47; //PID val for pitch
extern const double pid_p_Yaw=3.0, pid_i_Yaw=0.04, pid_d_Yaw=0.0; //PID val for yaw

extern const int pid_max_Roll = 400; //Max PID output for roll
extern const int pid_max_Pitch = 400; //Max PID output for pitch
extern const int pid_max_Yaw = 400; //Max PID output for yaw


MPU6050 mpu (Wire);

float pid_i_mem_Roll, pid_setpoint_Roll, gyro_input_Roll, pid_output_Roll, pid_last_d_error_Roll; //Roll PID calcul value
float pid_i_mem_Pitch, pid_setpoint_Pitch, gyro_input_Pitch, pid_output_Pitch, pid_last_d_error_Pitch; //Pitch PID calcul value
float pid_i_mem_Yaw, pid_setpoint_Yaw, gyro_input_Yaw, pid_output_Yaw, pid_last_d_error_Yaw; //Yaw PID calcul value


float pitch_val;
float roll_val;
float yaw_val;

float ESC_val_FL = 1000, ESC_val_FR  = 1000, ESC_val_BL  = 1000, ESC_val_BR  = 1000;

bool isReady = false;

void setup() {
  Serial.begin(115200);

  Serial.println("Seting up mpu6050");
  mpu.Initialize();
  mpu.SetGyroOffsets(GyroX,GyroY,GyroZ); //Seting the gyrooffset from the Config.h file
  Serial.println("");
  Serial.println("Starting calibration...");
  mpu.Calibrate();

  Serial.println("Seting up Reciver");

  pinMode(ThrotlePin, INPUT);
  pinMode(RollPin, INPUT);
  pinMode(PitchPin, INPUT);
  pinMode(YawPin, INPUT);

  Serial.println("Seting up Done");
  Serial.println("Add throttle to begin");
  isReady = true;
}

void loop() {
  if(!isReady) return;//Wait for ready
  
  //Get the current throttle value
  int _throttle = map(pulseIn(ThrotlePin,HIGH),throttleMin,throttleMax,escIdle,escMax);


    //Get the new gyro value
  mpu.Execute();
  
  //Set the angles in an array
  float mpuAngles[3] = {mpu.GetAngX(), mpu.GetAngY(), mpu.GetAngZ()};

  //Set the new gyro Value based on the axis
  gyro_input_Roll = mpuAngles[0] * mappedAxis[0][0] + mpuAngles[1] * mappedAxis[0][1] + mpuAngles[2] * mappedAxis[0][2];
  gyro_input_Pitch = mpuAngles[0] * mappedAxis[1][0] + mpuAngles[1] * mappedAxis[1][1] + mpuAngles[2] * mappedAxis[1][2];
  gyro_input_Yaw = mpuAngles[0] * mappedAxis[2][0] + mpuAngles[1] * mappedAxis[2][1] + mpuAngles[2] * mappedAxis[2][2]; 

  if(_throttle <= escMin - 50)
  {
    ESC_val_FL = escIdle;
    ESC_val_FR = escIdle;
    ESC_val_BL = escIdle;
    ESC_val_BR = escIdle;
    return;
  }

  //Set the new setpoint from the controller
  pid_setpoint_Roll = map(pulseIn(RollPin, HIGH),rollMin,rollMax,rollMinAngle,rollMaxAngle);
  pid_setpoint_Pitch = map(pulseIn(PitchPin, HIGH),pitchMin,pitchMax,pitchMinAngle,pitchMaxAngle);
  pid_setpoint_Yaw = pid_setpoint_Yaw + map(pulseIn(YawPin, HIGH),yawMin,yawMax,yawMinAngle,yawMaxAngle) * yawStrenght;

  //Make sure the pid_setpoint_Yaw is in the range of the MPU6050
  if (pid_setpoint_Yaw > 180) pid_setpoint_Yaw = -180 + (pid_setpoint_Yaw - 180);
  if (pid_setpoint_Yaw < -180) pid_setpoint_Yaw = 180 + (pid_setpoint_Yaw + 180);

  //Compute the pid values
  pidCompute();

  //Seting the new ESC val from the pid output
  ESC_val_FL = _throttle + pid_output_Roll + pid_output_Pitch + pid_output_Yaw;
  ESC_val_FR = _throttle - pid_output_Roll + pid_output_Pitch - pid_output_Yaw;
  ESC_val_BL = _throttle + pid_output_Roll - pid_output_Pitch - pid_output_Yaw;
  ESC_val_BR = _throttle - pid_output_Roll - pid_output_Pitch + pid_output_Yaw;
  
  
  
  
  //Make sure motor val ar in range
  if (ESC_val_FL < escMin) ESC_val_FL = escMin;
  if (ESC_val_FR < escMin) ESC_val_FR = escMin;
  if (ESC_val_BL < escMin) ESC_val_BL = escMin;
  if (ESC_val_BR < escMin) ESC_val_BR = escMin;

  if(ESC_val_FL > escMax)ESC_val_FL = escMax;
  if(ESC_val_FR > escMax)ESC_val_FR = escMax;  
  if(ESC_val_BL > escMax)ESC_val_BL = escMax;
  if(ESC_val_BR > escMax)ESC_val_BR = escMax; 
  
  //Print values
 /* 
 Serial.print(pid_output_Roll);
  Serial.print("\t");
  Serial.print(pid_output_Pitch);
  Serial.print("\t");
  Serial.print(pid_output_Yaw);
  Serial.println("");
*/
  Serial.print(pid_setpoint_Roll);
  Serial.print("\t");
  Serial.print(pid_setpoint_Pitch);
  Serial.print("\t");
  Serial.print(pid_setpoint_Yaw);
  Serial.println("");

}
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


//Compute all of the pid values

//Custom map function to map with and to float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}