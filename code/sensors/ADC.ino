#include "ADS1115.h"
#include "ADC.h"

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS); 

void ads_setup() {                
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip
}

void readWP(AnalogData ads_data[NO_WP_SENSORS]){
    adc0.setMode(ADS1115_CFG_MODE_BIT);
    adc0.setGain(ADS1115_PGA_4P096 );
    adc0.setMultiplexer(ADS1115_MUX_P0_NG); // set the mux anyway
    delay(30);
    adc0.setMode(ADS1115_MODE_SINGLESHOT);
    ads_data[0].adc = 0;
    ads_data[0].value =  (float)adc0.getConversionP0GND() * 6.25 -240;
    delay(50);    
    adc0.setMode(ADS1115_CFG_MODE_BIT);
    adc0.setGain(ADS1115_PGA_6P144 );
    adc0.setMultiplexer(ADS1115_MUX_P1_NG);
    delay(30);
    adc0.setMode(ADS1115_MODE_SINGLESHOT);
    ads_data[1].adc = 1;
    ads_data[1].value = (float)adc0.getConversionP1GND() * 4.43413 + 5600;  // counts up to 16-bits
}
