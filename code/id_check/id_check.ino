#include <EEPROM.h>
// print the node's ID from EEPROM once - for checking it is OK
#define IO_PIN 2

void setup() {
  
  Serial.begin(9600);
  pinMode(IO_PIN, OUTPUT);
  
  digitalWrite(IO_PIN, HIGH);
  Serial.println("This program will print the ID from the AVR");
  Serial.print("The address is:");
  Serial.println(EEPROM.read(0), HEX);
  Serial.print("The mode is:");
  Serial.println(EEPROM.read(1), HEX);
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}

void loop() {
  
  
}
