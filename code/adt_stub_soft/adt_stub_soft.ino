
#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5
#define I2C_TIMEOUT 10
#include <SoftI2CMaster.h>

#define TMP_ADDR1 0x48
#define TMP_ADDR2 0x4B

#define DEBUG 1

void setup() {
 byte b1, b2;
  Serial.begin(9600);
  i2c_init();
 
}

float tmp_read_soft(byte device){
  byte b1, b2;
  float t;
  i2c_start((device << 1) | I2C_READ); 
  b1 = i2c_read(0); 
  b2 = i2c_read(1);
  i2c_stop();
  Serial.print("B1: ");
  Serial.println( b1 );
  Serial.print("B2: ");
  Serial.println( b2);
  t = b1 * 256   + b2;
  if (b1 && 0x80){
    t = t-512;
  }
  
  return t/128;
}

void loop() {
  // put your main code here, to run repeatedly:
 
  Serial.println(tmp_read_soft(TMP_ADDR2));
  delay(1000);
}
