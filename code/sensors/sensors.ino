// Setup the pin muxing for the fake I2C
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7
#define I2C_TIMEOUT 10
#include <SoftI2CMaster.h>
#include "buffer.h"
#include "pb_encode.h"

//Real I2C
#include <Wire.h>

//Addresses
#define TMP_ADDR1 0x48
#define TMP_ADDR2 0x4B
#define ACCEL_ADDR1 (0x53) // Device address as specified in data sheet
#define ACCEL_ADDR2 0x1D

//ADX345 Memory Addresses
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;

char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

bool DEBUG = 0;




void setup(void) {
  Serial.begin(9600);
  if (DEBUG){
    Serial.println("Starting real I2C...");
  }
  Wire.begin();
  if (DEBUG){
    Serial.print("Ok\n Starting fake I2C...");
  }
  i2c_init();
  if (DEBUG){
    Serial.println("Ok");
  }
}

//I2C Stuff

void read_chain(){
  Rs485 message;
  message.dst = 0;
  message.type = Rs485_Type_DATA;
  message.has_sensor = true;
  message.sensor = Rs485_Sensor_TA_CHAIN;
  message.ow_count = 0;
  message.ad_count = 0;
  message.tad_count = 4;
  
  readTAD(TMP_ADDR1, ACCEL_ADDR1, false, 1, &message.tad[0]);
  readTAD(TMP_ADDR2, ACCEL_ADDR2, false, 2, &message.tad[1]);
  readTAD(TMP_ADDR1, ACCEL_ADDR1, true, 3, &message.tad[2]);
  readTAD(TMP_ADDR2, ACCEL_ADDR2, true, 4, &message.tad[3]);
  
  pb_ostream_t stream2 = {&serialWriter,0,512,0,0};
  bool status = pb_encode(&stream2, Rs485_fields, &message);
//  Serial.print(" ");
//  Serial.print(stream2.bytes_written);
//  Serial.println("");
}

bool serialWriter(pb_ostream_t *stream, const uint8_t *buf, size_t count) {  
  while (count--)
    Serial.write(*buf++);
  
  return true;
}

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
  if (DEBUG){
    Serial.print("WIRE: ");
    Serial.print(device);
    Serial.print(":");
    Serial.print("B1: ");
    Serial.println( b1 );
    Serial.print("B2: ");
    Serial.println( b2);
  }
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
  if (DEBUG){
    Serial.print("SOFT: ");
    Serial.print(device);
    Serial.print(":");
    Serial.print("B1: ");
    Serial.println( b1 );
    Serial.print("B2: ");
    Serial.println( b2);
  }
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

void loop(void) {
  read_chain();
  delay(5000);
}

