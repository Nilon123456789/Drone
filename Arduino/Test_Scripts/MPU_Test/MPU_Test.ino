#include <TinyMPU6050.h>

//Piezo
#define PiezoPin 3

MPU6050 mpu (Wire);

bool isready = false;

void setup() {
    Serial.begin(115200);
    tone(PiezoPin, 3000, 250);
    mpu.Initialize();
    mpu.Calibrate();
    Serial.println("");

    tone(PiezoPin, 3000, 250);
    delay(100);
    tone(PiezoPin, 3000, 250);

    Serial.println("X \t Y \t Z"); 
    isready = true;
}

void loop() {
    if(!isready) return;//Wait for ready

    mpu.Execute();

    Serial.print(mpu.GetAngX());
    Serial.print("\t");
    Serial.print(mpu.GetAngY());
    Serial.print("\t");
    Serial.println(mpu.GetAngZ());
}