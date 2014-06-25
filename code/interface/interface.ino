//OneWire Config
#include <OneWire.h>
int OW_PIN = 10;

// Setup the pin muxing for the fake I2C
#define SDA_PORT PORTC
#define SDA_PIN 0
#define SCL_PORT PORTC
#define SCL_PIN 1
#define I2C_TIMEOUT 10
#include <SoftI2CMaster.h>

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


OneWire  ds(OW_PIN);  // on pin 10


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
  //Setup Accels
  setup_accel_soft(ACCEL_ADDR1);
  setup_accel_soft(ACCEL_ADDR2);
  setup_accel_wire(ACCEL_ADDR1);
  setup_accel_wire(ACCEL_ADDR2);
}

float tmp_read_wire(byte device){
  byte b1, b2;
  float t;
  Wire.requestFrom((int)device, 2);
  b1 = Wire.read();
  b2 = Wire.read();
  if (DEBUG){
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

void setup_accel_soft(byte addr){
  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo_soft(addr, DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo_soft(addr, POWER_CTL, 0x08);
  if (DEBUG){
    Serial.println("ADXL setup");
  }
}

void readAccel_soft(byte addr, double data[]) {

  uint8_t howManyBytesToRead = 6;
  byte _buff[6];
  double res[2];
  readFrom_soft( addr, DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

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
  if (DEBUG){
    Serial.println("ADXL setup");
  }
}

void readAccel_wire(byte addr, double data[]) {

  uint8_t howManyBytesToRead = 6;
  byte _buff[6];
  double res[2];
  readFrom_wire( addr, DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

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

//OneWire Stuff

void read_1W(){
  byte addr[8];
  while(ds.search(addr)){
    read_1W_addr(addr);
  }
  ds.reset_search();
  delay(250);
}

void read_1W_addr(byte addr[8]){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius;
  if (DEBUG){
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      if (DEBUG){
        Serial.println("CRC is not valid!");
      }
      return;
  }
  
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      if(DEBUG){
        Serial.println("  Chip = DS18S20");  // or old DS1820
      }
      type_s = 1;
      break;
    case 0x28:
      if(DEBUG){
        Serial.println("  Chip = DS18B20");
      }
      type_s = 0;
      break;
    case 0x22:
      if(DEBUG){
        Serial.println("  Chip = DS1822");
      }
      type_s = 0;
      break;
    default:
      if(DEBUG){
        Serial.println("Device is not a DS18x20 family device.");
      }
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  if (DEBUG){
      Serial.print("  Data = ");
      Serial.print(present,HEX);
      Serial.print(" ");
  }
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    if (DEBUG){
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
  }
  if (DEBUG){
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
  }  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  if(DEBUG){
    Serial.print("  Temperature = ");
    Serial.print(celsius);
    Serial.print(" Celsius, ");
  }
}


void loop(void) {
  read_1W();
  read_chain();
  }
