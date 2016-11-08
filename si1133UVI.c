#include <SoftwareSerial.h>


void setup() {
  Serial.begin(9600);
  Serial.println("testeo");
  Serial.println("OK!");
}

void loop() {
  
  Serial.println("===================");
  Serial.print("FULL-IR: "); 
  Serial.print("UV: ");
  delay(1000);
}
