/* 
 * i2c.h: utility methods for reading and writing to software and wire based i2c busses
 */
#ifndef _I2C_H_
#define _I2C_H_

#include <Arduino.h>

//write the given byte to the device
void writeTo_soft(byte device, byte address, byte val);

// Reads num bytes starting from address register on device in to _buff array
void readFrom_soft(byte device, byte address, int num, byte _buff[]);

//write the given byte to the device
void writeTo_wire(byte device, byte address, byte val);

// Reads num bytes starting from address register on device in to _buff array
void readFrom_wire(byte device, byte address, int num, byte _buff[]);

#endif
