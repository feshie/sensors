#include "TAD.h"
#include "i2c.h"

float tmp_read_wire(byte device);
float tmp_read_soft(byte device);
void setup_accel_soft(byte addr);
void read_accel_soft(byte addr, TempAccelData* tad);
void setup_accel_wire(byte addr);
void read_accel_wire(byte addr, TempAccelData* tad);

void readTAD(byte tempAddr, byte accelAddr, boolean soft, int id, TempAccelData *tad) {
  tad->id = id;
  
  if (soft) {
    tad->temp = tmp_read_soft(tempAddr);
    setup_accel_soft(accelAddr);
    read_accel_soft(accelAddr, tad);
  } else { 
    tad->temp = tmp_read_wire(tempAddr);
    setup_accel_wire(accelAddr);
    read_accel_wire(accelAddr, tad);
  }
}

float tmp_read_wire(byte device){
  byte b1, b2;
  float t;
  Wire.requestFrom((int)device, 2);
  b1 = Wire.read();
  b2 = Wire.read();

  t = b1 * 256   + b2;
  if (b1 && 0x80){
    t = t-512;
  }
  
  return (t/128);
}

float tmp_read_soft(byte device){
  byte b1, b2;
  float t;
  i2c_start((device << 1) | I2C_READ); 
  b1 = i2c_read(0); 
  b2 = i2c_read(1);
  i2c_stop();

  t = b1 * 256   + b2;
  if (b1 && 0x80){
    t = t-512;
  }
  
  return t/128;
}

void setup_accel_soft(byte addr){
  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo_soft(addr, DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo_soft(addr, POWER_CTL, 0x08);
}

void read_accel_soft(byte addr, TempAccelData* tad) {
  uint8_t howManyBytesToRead = 6;
  byte _buff[6];
  double res[2];
  readFrom_soft( addr, DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  double x = (((int)_buff[1]) << 8) | _buff[0];
  double y = (((int)_buff[3]) << 8) | _buff[2];
  double z = (((int)_buff[5]) << 8) | _buff[4];

  // Conversion from http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
  tad->roll  = (atan2(-y, z)*180.0)/M_PI;
  tad->pitch = (atan2(x, sqrt(y*y + z*z))*180.0)/M_PI;
}

void setup_accel_wire(byte addr){
  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo_wire(addr, DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo_wire(addr, POWER_CTL, 0x08);
}

void read_accel_wire(byte addr, TempAccelData* tad) {

  uint8_t howManyBytesToRead = 6;
  byte _buff[6];
  double res[2];
  readFrom_wire( addr, DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  double x = (((int)_buff[1]) << 8) | _buff[0];
  double y = (((int)_buff[3]) << 8) | _buff[2];
  double z = (((int)_buff[5]) << 8) | _buff[4];

  // Conversion from http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
  tad->roll  = (atan2(-y, z)*180.0)/M_PI;
  tad->pitch = (atan2(x, sqrt(y*y + z*z))*180.0)/M_PI;
}
