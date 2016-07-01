#include <EEPROM.h>
#include "sensors.h"
// Write the unique ID to the AVR's EEPROM
//Use a variation of the following to stop the eeprom being overwritten by a programmer
//avrdude -p atmega328p -c avrispv2 -P/dev/tty.usbmodem00079651 -U hfuse:w:0xD2:m

//Pin for toggling rx/tx
#define IO_PIN 2

void setup() {
  
  Serial.begin(9600);
  
//  digitalWrite(IO_PIN, HIGH);
//  Serial.println("This program will burn an ID to the eeprom of the AVR");
//
//  Serial.println("Please enter the ID that you would like to burn to this device\n");
//  Serial.println("NOTE: The value stored is the byte value of the ascii character entered");
//  Serial.print(">");
//  Serial.flush();
//  digitalWrite(IO_PIN, LOW);
//
//  delay(10);
//
//  char addr[1];
//  while(!Serial.available());
//  Serial.readBytes(addr, 1);
//  
//  digitalWrite(IO_PIN, HIGH);
//  Serial.println("");
//  Serial.print("Storing address ");
//  Serial.print((byte)addr[0]);
//  Serial.println(". If this is not correct power off now");
//  Serial.println("Writing in 10s");
//  delay(10000);
//  Serial.println("Burning address");
//  EEPROM.write(0,addr[0]);
//  Serial.print("Address Burnt:");
//  Serial.println(EEPROM.read(0));
//  Serial.flush();
//  digitalWrite(IO_PIN, LOW);

  EEPROM.write(EEPROM_ADDR_ID,0x22);
  EEPROM.write(EEPROM_SENSORS,04);
  
  pinMode(IO_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
  digitalWrite(IO_PIN, HIGH);
  Serial.println(EEPROM.read(0), HEX);
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}



