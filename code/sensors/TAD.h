/* 
 * TAD.h: read from the temperature and accelerometer sensors
 */
#ifndef _TAD_H_
#define _TAD_H_

#include <Arduino.h>
#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

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

//read the data from the given sensor into the TempAccelData structure
void readTAD(byte tempAddr, byte accelAddr, boolean soft, int id, TempAccelData *tad);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
