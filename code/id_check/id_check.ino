#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("This program will print the ID from the AVR");
  Serial.print("The address is:");
  Serial.print(EEPROM.read(0));
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
