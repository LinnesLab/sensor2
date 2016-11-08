/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
#include <Wire.h>
#include "SI1133.h"

SI1133 uv = SI1133();

void setup() {
  Serial.begin(9600);
  Serial.println("testeo");
  if (! uv.begin()) {
    Serial.println("no se encuentra el modulo");
    while (1);
  }
  Serial.println("OK!");
}

void loop() {
  
  Serial.println("===================");
  uint32_t UV = uv.readUV();
//  float IR = uv.readIR();

//  Serial.print("FULL-IR: ");  Serial.println(IR,BIN);
  Serial.print("UV: ");  Serial.println(UV,BIN);
Serial.print("UV: ");  Serial.println(UV);
  if(UV>545){
    Serial.println("UVI: 14");
  }
  else
  Serial.print("UVI: ");  Serial.println(0.0082*(0.00391*UV*UV+UV));
  uv.printOut();
  

  delay(1000);
}
