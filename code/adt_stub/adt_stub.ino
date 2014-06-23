#include <Wire.h>

#define TMP_ADDR 0x4b
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
    byte b1, b2;
  Wire.requestFrom(TMP_ADDR, 2);
  b1 = Wire.read();
  b2 = Wire.read();
  Serial.print("B1: ");
  Serial.println( b1);
  Serial.print("B2: ");
  Serial.println( b2);
  Serial.println("********");
}

void loop() {
  // put your main code here, to run repeatedly:
  byte b1, b2;
  float b3;
  Wire.requestFrom(TMP_ADDR, 2);
  b1 = Wire.read();
  b2 = Wire.read();
  Serial.print("B1: ");
  Serial.println( b1 );
  Serial.print("B2: ");
  Serial.println( b2);
  b3 = b1 * 256   + b2;
  if (b1 && 0x80){
    b3 = b3-512;
  }
  
  Serial.println(b3/128);
  delay(1000);
}
