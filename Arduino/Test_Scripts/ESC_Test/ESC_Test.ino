#include <Servo.h>


//ESC                                                        CW    CCW
#define EscPinFL 11 //CW                                       ╲  ╱
#define EscPinFR 10 //CCW                                        ▉     ↑ forward
#define EscPinBL 9 //CCW                                       ╱  ╲
#define EscPinBR 6 //CW                                     CCW    CW

//Remote
#define ThrotlePin A0

/******ESC Values******/
const int escIdle = 1000; //ESC idle speed
const int escMin = 1100; //ESC min working speed (for all esc)
const int escMax = 2000; //ESC max working spees (for all esc)

/******RC Values******/
const int throttleMin=995, throttleMax=1988;

Servo ESC_FL, ESC_FR, ESC_BL, ESC_BR;

bool isReady = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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

  for(int i; i < 10; i++) {//Delay with feedback to let the esc boot
    Serial.print('*');
    delay(1000);
  }
  Serial.println("");

  Serial.println("Seting up Reciver");

  pinMode(ThrotlePin, INPUT);


  isReady = true;
  Serial.println("Ready");
}

void loop() {
  if(!isReady) return;//Wait for ready

  int _throttle = map(pulseIn(ThrotlePin,HIGH),throttleMin,throttleMax,escIdle,escMax);
  Serial.println(_throttle);
  //Write the ESC values
  ESC_FL.writeMicroseconds(_throttle);
  ESC_FR.writeMicroseconds(_throttle);
  ESC_BL.writeMicroseconds(_throttle);
  ESC_BR.writeMicroseconds(_throttle);
}