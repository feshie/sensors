#include <Wire.h>
// https://github.com/jenschr/Arduino-libraries/blob/master/ADXL345/examples/ADXL345_no_library/BareBones_ADXL345.pde
#define DEVICE1 (0x53) // Device address as specified in data sheet
#define DEVICE2 0x1D
#define DEBUG 0



char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;

char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600); // start serial for output. Make sure you set your Serial Monitor to the same!
  setup_accel_wire(DEVICE2);
}

void setup_accel_wire(byte addr){
  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(addr, DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(addr, POWER_CTL, 0x08);
  if (DEBUG){
    Serial.println("ADXL setup");
  }
}


void loop()
{
  double data[2];
  readAccel(DEVICE2, data); // read the x/y/z tilt
  Serial.print(data[0]);
  Serial.print(":");
  Serial.println(data[1]);
  delay(1000); // only read every 0,5 seconds
}

void readAccel(byte addr, double data[]) {

  uint8_t howManyBytesToRead = 6;
  byte _buff[6];
  double res[2];
  readFrom( addr, DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  double x = (((int)_buff[1]) << 8) | _buff[0];
  double y = (((int)_buff[3]) << 8) | _buff[2];
  double z = (((int)_buff[5]) << 8) | _buff[4];
  if (DEBUG){
    Serial.print("x: ");
    Serial.print( x );
    Serial.print(" y: ");
    Serial.print( y );
    Serial.print(" z: ");
    Serial.println( z );
  }
  // Conversion from http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
  double roll, pitch;
  roll  = (atan2(-y, z)*180.0)/M_PI;
  pitch = (atan2(x, sqrt(y*y + z*z))*180.0)/M_PI;
  data[0] = pitch;
  data[1] = roll;
}

void writeTo(byte device, byte address, byte val) {
  Wire.beginTransmission(device); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte device, byte address, int num, byte _buff[]) {
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
