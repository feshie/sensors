#include <EEPROM.h>

#define IO_PIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IO_PIN, OUTPUT);
  
  digitalWrite(IO_PIN, HIGH);
  Serial.println("This program will print the ID from the AVR");
  Serial.print("The address is:");
  Serial.println(EEPROM.read(0));
  Serial.print("The mode is:");
  Serial.println(EEPROM.read(1));
  Serial.flush();
  digitalWrite(IO_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
