/*
 * This is the Setup script of the Drone
 * Save the gyro and reciver vales.
 * Don't forget to check the pin of the
 * component and the wiring. Also it is
 * sugested that you run the balance script
 * to setup the pid values. This script must
 * be run with the serial monitor opened with
 * the baudrate set to 115200
 * 
 * 
 * Made by Nils Lahaye 2022
 */

/************************************/
/*         Included Libraries       */
/************************************/

#include <TinyMPU6050.h>

/************************************/
/*              Options             */
/************************************/

#define RCSETUP //comment this line to skip the part
#define MPUSETUP //comment this line to skip the part

/************************************/
/*              Pinout              */
/************************************/

//Remote
#define ThrotlePin A0
#define RollPin A1
#define PitchPin A2  
#define YawPin A3
#define Extra1Pin 4
#define Extra2Pin 5

/******************************************/
/*            GLOBAL VARIABLES            */
/******************************************/

MPU6050 mpu (Wire);

int throttleMin=0L, throttleMax=0L;
int rollMin=0L, rollMax=0L;
int pitchMin=0L, pitchMax=0L;
int yawMin=0L, yawMax=0L;
int extra1Min=0L, extra1Max=0L;
int extra2Min=0L, extra2Max=0L;

float GyroX=0L, GyroY=0L, GyroZ=0L;

/*******************************/
/*            SETUP            */
/*******************************/

void setup() {
  //Starting Serial
  Serial.begin(115200);
  
  Serial.println(F("==================================================="));
  Serial.println(F("Setting up"));
  Serial.println(F("==================================================="));
  
  #ifdef RCSETUP
    RcSetup(); //Remote Setup
  #endif

  #ifdef MPUSETUP
    MpuSetup(); //MPU6050 Setup
  #endif
  
  Serial.println(F("==================================================="));
  Serial.println(F("Setting done"));
  Serial.println(F("Copy all of the value in the config.h of the main project"));
  Serial.println(F("==================================================="));
  
  PrintAll(); //Print all of the values
}

void RcSetup() {
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println("Seting up Remote");
  Serial.println(F("==================================================="));

  WaitForKey();//Wait for user to be ready

  //Get min max for the throttle
  delay(1000);
  throttleMin = GetMinMax(ThrotlePin,"throttle",false);
  throttleMax = GetMinMax(ThrotlePin,"throttle",true);

  Serial.println(F(""));
  Serial.println(F(""));

  //Get min max for the roll
  delay(1000);
  rollMin = GetMinMax(RollPin,"roll",false);
  rollMax = GetMinMax(RollPin,"roll",true);

  Serial.println(F(""));
  Serial.println(F(""));

  //Get min max for the pitch
  delay(1000);
  pitchMin = GetMinMax(PitchPin,"pitch",false);
  pitchMax = GetMinMax(PitchPin,"pitch",true);

  Serial.println(F(""));
  Serial.println(F(""));

  //Get min max for the yaw
  delay(1000);
  yawMin = GetMinMax(YawPin,"yaw",false);
  yawMax = GetMinMax(YawPin,"yaw",true);

  //Get min max for the Extra 1
  delay(1000);
  extra1Min = GetMinMax(Extra1Pin,"Extra 1",false);
  extra1Max = GetMinMax(Extra1Pin,"Extra 2",true);

  //Get min max for the Extra 2
  delay(1000);
  extra2Min = GetMinMax(Extra2Pin,"Extra 2",false);
  extra2Max = GetMinMax(Extra2Pin,"Extra 2",true);

  
  
  Serial.println("Remote Min Max:");

  Serial.print("\t");

  String varName = "extern const int ";

  String throttleString = "throttleMin=" + String(throttleMin) + ", throttleMax=" + String(throttleMax) + ";"; 
  Serial.println(varName + throttleString);
  
  Serial.print("\t");
  String rollString = "rollMin=" + String(rollMin) + ", rollMax=" + String(rollMax) + ";"; 
  Serial.println(varName + rollString);

  Serial.print("\t");
  String pitchString = "pitchMin=" + String(pitchMin) + ", pitchMax=" + String(pitchMax) + ";"; 
  Serial.println(varName + pitchString);

  Serial.print("\t");
  String yawString = "yawMin=" + String(yawMin) + ", yawMax=" + String(yawMax) + ";"; 
  Serial.println(varName + yawString);

  Serial.print("\t");
  String extra1String = "extra1Min=" + String(extra1Min) + ", extra1Max=" + String(extra1Max) + ";"; 
  Serial.println(varName + extra1String);

  Serial.print("\t");
  String extra2String = "extra2Min=" + String(extra2Min) + ", extra2Max=" + String(extra2Max) + ";"; 
  Serial.println(varName + extra2String + "\n");
}

void MpuSetup() {
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println("Seting up MPU6050");
  Serial.println(F("==================================================="));

  WaitForKey();//Wait for user to be ready

  mpu.Initialize();
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");

  //Set offsets
  GyroX = mpu.GetGyroXOffset();
  GyroY = mpu.GetGyroYOffset();
  GyroZ = mpu.GetGyroXOffset();
  
  Serial.println("Remote Min Max:");

  Serial.print("\t");

  String mpuOffsetString = "extern const float GyroX=" + String(GyroX) + ", GyroY=" + String(GyroY) + ", GyroZ=" + String(GyroZ) + ";"; 
  Serial.println(mpuOffsetString);
  
}

long result = 0;
int j = 0;
int GetMinMax(int pin, String joyStick, bool Max) {
   
   j = 0; //Make sure j is = 0
   result = 0; //Make sure result is = 0
  
  if(Max) Serial.println("\t Keep the " + joyStick + " on the max position");
  else Serial.println("\t Keep the " + joyStick + " on the min position");
  
  delay(5000); //Delay to give time to place the joystick
    
  Serial.print("\t Capturing...");
    
  delay(500); //delay for fun
    
  Serial.print(" >");
  for (int i; i < 50; i++)
  {
     result += pulseIn(pin,HIGH); //Read value and add it
     j++;
      
     if(i % 10 == 0) Serial.print("*");
  }
  Serial.println("");
  return result / j; // Return the avrage
}

void PrintAll() {
  
  #ifdef RCSETUP
    Serial.println("Remote Min Max:");

    Serial.print("\t");

    String varName = "extern const int ";

    String throttleString = "throttleMin=" + String(throttleMin) + ", throttleMax=" + String(throttleMax) + ";"; 
    Serial.println(varName + throttleString);
    
    Serial.print("\t");
    String rollString = "rollMin=" + String(rollMin) + ", rollMax=" + String(rollMax) + ";"; 
    Serial.println(varName + rollString);

    Serial.print("\t");
    String pitchString = "pitchMin=" + String(pitchMin) + ", pitchMax=" + String(pitchMax) + ";"; 
    Serial.println(varName + pitchString);

    Serial.print("\t");
    String yawString = "yawMin=" + String(yawMin) + ", yawMax=" + String(yawMax) + ";"; 
    Serial.println(varName + yawString);


  Serial.print("\t");
  String extra1String = "extra1Min=" + String(extra1Min) + ", extra1Max=" + String(extra1Max) + ";"; 
  Serial.println(varName + extra1String);

  Serial.print("\t");
  String extra2String = "extra2Min=" + String(extra2Min) + ", extra2Max=" + String(extra2Max) + ";"; 
  Serial.println(varName + extra2String + "\n");
  #endif

  Serial.println();
  
  #ifdef MPUSETUP
    Serial.println("Remote Min Max:");

    Serial.print("\t");

    String mpuOffsetString = "extern const float GyroX=" + String(GyroX) + ", GyroY=" + String(GyroY) + ", GyroZ=" + String(GyroZ) + ";"; 
    Serial.println(mpuOffsetString);
  #endif
}

void WaitForKey() {
  
  Serial.println(F("\nSend any character to begin : "));

  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

}

//Intentonaly left blank
void loop(){}
