#define ThrotlePin A0
#define RollPin A1
#define PitchPin A2  
#define YawPin A3
#define Extra1Pin A4
#define Extar2Pin A5

void setup() {
  Serial.begin(115200);
  Serial.println("Throtle \t Roll \t Pitch \t Yaw \t CH5 \t CH6"); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(pulseIn(ThrotlePin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(RollPin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(PitchPin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(YawPin,HIGH));
  Serial.print("\t");
  Serial.print(pulseIn(Extra1Pin,HIGH));
  Serial.print("\t");
  Serial.println(pulseIn(Extra1Pin,HIGH));
}
