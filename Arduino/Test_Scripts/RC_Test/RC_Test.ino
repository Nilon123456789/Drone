#define ThrotlePin A0
#define RollPin A1
#define PitchPin A2  
#define YawPin A3

void setup() {
  Serial.begin(115200);
  Serial.println("Throtle \t Roll \t Pitch \t Yaw"); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(pulseIn(ThrotlePin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(RollPin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(PitchPin,HIGH));
  Serial.print("\t");
  Serial.println(pulseIn(YawPin,HIGH));
}
