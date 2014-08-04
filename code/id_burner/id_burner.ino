#include <EEPROM.h>

//Pin for toggling rx/tx
#define IO_PIN 2

void setup() {
  // put your setup code here, to run once:
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

  EEPROM.write(0,0x01);
  
  pinMode(IO_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
  digitalWrite(IO_PIN, HIGH);
  Serial.println(EEPROM.read(0));
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}
