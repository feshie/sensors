#include <I2Cdev.h>

// I2C device class (I2Cdev) demonstration Arduino sketch for ADS1115 class
// Example of reading two differential inputs of the ADS1115 and showing the value in mV
// 06 May 2013 by Frederick Farzanegan (frederick1@farzanegan.org)
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2013-05-13 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include <Wire.h>
#include "ADS1115.h"

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS); 
#define IO_PIN 2
void setup() {                
    pinMode(IO_PIN, OUTPUT);
    digitalWrite(IO_PIN, HIGH);
    Serial.begin(9600); // initialize serial communication 
    Serial.println("Starting ADS Stub");
    Wire.begin();  // join I2C bus
    Serial.println("Initializing I2C devices..."); 
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip
    
    Serial.println("Testing device connections...");
    Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
    
}

void loop() 
  {

    // Sensor is on P0/N1 (pins 4/5)
    Serial.println("Sensor 1 ************************");
    adc0.setMode(ADS1115_CFG_MODE_BIT);
    adc0.setGain(ADS1115_PGA_4P096 );
    // Get the number of counts of the accumulator
    Serial.print("Counts for sensor 1 is:");
    adc0.setMultiplexer(ADS1115_MUX_P0_NG); // set the mux anyway
    delay(30);
    // The below method sets the mux and gets a reading.
    adc0.setMode(ADS1115_MODE_SINGLESHOT);
    int sensorOneCounts=adc0.getConversionP0GND();  // counts up to 16-bits  
    Serial.println(sensorOneCounts);
    
        Serial.println("Sensor 2 ************************");
    adc0.setMode(ADS1115_CFG_MODE_BIT);
    // Set the gain (PGA) +/- 6.144v
    adc0.setGain(ADS1115_PGA_6P144 );
       Serial.print("Counts for sensor 2 is:");
    adc0.setMultiplexer(ADS1115_MUX_P1_NG);
    delay(30);
    adc0.setMode(ADS1115_MODE_SINGLESHOT);
    // The below method sets the mux and gets a reading.
    int sensorTwoCounts=adc0.getConversion();  // counts up to 16-bits  
    Serial.println(sensorTwoCounts);
    
    Serial.println();
    
    delay(1000);
  }
  

  


