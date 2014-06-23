#include <Wire.h>

#define ACCEL_ADDR 0x53
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Waiting");
  delay(5000);
  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x31);
  Wire.write(0x00); //2g mode
  Wire.endTransmission();
  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x2D);
  Wire.write(0x80); //enable readings
  Wire.endTransmission();
  Serial.println("Starting");
  int x1,x2,y1,y2,z1,z2;
  double x, y, z;
  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x32);
  Wire.requestFrom(ACCEL_ADDR, 6);
  x1 = Wire.read();
  x2 = Wire.read();
  y1 = Wire.read();
  y2 = Wire.read();
  z1 = Wire.read();
  z2 = Wire.read();
  Wire.endTransmission();
  x = x2 * 255 + x1;
  y = y2 * 255 + y1;
  z = z2 * 255 + y1;
  Serial.print("X:");
  Serial.println(x);
  Serial.print("Y:");
  Serial.println(y);
  Serial.print("Z:");
  Serial.println(z);
  Serial.println("Done");
 
 
 // Conversion from http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
   double roll, pitch;
    roll  = (atan2(-y, z)*180.0)/M_PI;
    pitch = (atan2(x, sqrt(y*y + z*z))*180.0)/M_PI;
    Serial.print(pitch);
    Serial.print(":");
    Serial.print(roll);
}

void loop() {
  // put your main code here, to run repeatedly:

}
