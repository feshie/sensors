/* 
 * OW.h: utility methods for reading and writing to software onewire busses
 */
#ifndef _OW_H_
#define _OW_H_

#include <Arduino.h>
#include "buffer.h"

#define OW_PIN  A2
#define NO_OW_SENSORS 5

void readOW( OwData owds[NO_OW_SENSORS]);



#endif
