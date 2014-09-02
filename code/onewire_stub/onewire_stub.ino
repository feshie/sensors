#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

int OW_PIN = A2;
bool DEBUG = 1;
#define IO_PIN 2


OneWire  ds(OW_PIN);  // on pin 10


void setup(void) {
  Serial.begin(9600);
    digitalWrite(IO_PIN, HIGH);
  read_1W();
  Serial.println("*****");
}

void read_1W(){
  byte addr[8];
  if (DEBUG){
    Serial.println("Starting Search");
  }
  while(ds.search(addr)){
    read_1W_addr(addr);
    delay(500);
  }
  
  ds.reset_search();
  if (DEBUG){
     Serial.println("Loop complete");
  }
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
  }

  // convert the data to actual temperature

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
  Serial.print(addr[7], HEX);
  Serial.print(",");
  Serial.println(celsius);


}


void loop(void) {
  read_1W();
  }

