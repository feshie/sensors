#include "i2c.h"

void writeTo_soft(byte device, byte address, byte val) {
  i2c_start((device << 1) | I2C_WRITE); // start transmission to device
  i2c_write(address); // send register address
  i2c_write(val); // send value to write
  i2c_stop(); // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom_soft(byte device, byte address, int num, byte _buff[]) {
  i2c_start((device << 1) | I2C_WRITE); // start transmission to device
  i2c_write(address); // sends address to read from
  i2c_rep_start((device << 1) | I2C_READ);

  int i = 0;
  while(i < num) // device may send less than requested (abnormal)
  {
    if (i < (num -1)){
      _buff[i] = i2c_read(0); // receive a byte
    }else{
      _buff[i] = i2c_read(1);
    }
    i++;
  }
  i2c_stop(); // end transmission
}

void writeTo_wire(byte device, byte address, byte val) {
  Wire.beginTransmission(device); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom_wire(byte device, byte address, int num, byte _buff[]) {
  Wire.beginTransmission(device); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(); // end transmission

  Wire.requestFrom((int)device, num); // request 6 bytes from device

  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
  {
    _buff[i] = Wire.read(); // receive a byte
    i++;
  }
}
