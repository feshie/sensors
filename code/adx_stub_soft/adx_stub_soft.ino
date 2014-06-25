


// https://github.com/jenschr/Arduino-libraries/blob/master/ADXL345/examples/ADXL345_no_library/BareBones_ADXL345.pde
#define DEVICE1 (0x53) // Device address as specified in data sheet
#define DEVICE2 0x1D

#define DEBUG 1

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5
#define I2C_TIMEOUT 10
#include <SoftI2CMaster.h>

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
  Serial.begin(9600); // start serial for output. Make sure you set your Serial Monitor to the same!
  Serial.print("Init: ");
  Serial.println(i2c_init());
  setup_accel_soft(DEVICE2);
}

void setup_accel_soft(byte addr){
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
  readAccel_soft(DEVICE2, data); // read the x/y/z tilt
  Serial.print(data[0]);
  Serial.print(":");
  Serial.println(data[1]);
  delay(1000); // only read every 0,5 seconds
}

void readAccel_soft(byte addr, double data[]) {

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
  i2c_start((device << 1) | I2C_WRITE); // start transmission to device
  i2c_write(address); // send register address
  i2c_write(val); // send value to write
  i2c_stop(); // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte device, byte address, int num, byte _buff[]) {
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
