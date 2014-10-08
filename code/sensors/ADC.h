/* 
 * ADC.h: utility methods for using the ADS device
 */
#ifndef _ADC_H_
#define _ADC_H_

#include <Arduino.h>
#include "buffer.h"

#define NO_WP_SENSORS 2

void ads_setup(void);
void readWP(AnalogData ads[NO_WP_SENSORS]);




#endif
