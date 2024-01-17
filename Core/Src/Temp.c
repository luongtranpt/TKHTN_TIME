#include "Temp.h"
#include "main.h" 

extern ADC_HandleTypeDef hadc1;
extern uint16_t adcv;	
extern uint16_t flag_temp;
// chuong trinh ngat ADC 


// Ham chuyen doi ra nhiet do 
double temp_convert(uint16_t adc1){  
double temp1;
   // if ( adc1 >= 1365 || adc1 <= 1910 )
			temp1 = (double) adc1 * 100 *3.3 / (4.4 * 4096); 
   // if (adc1 		
	return temp1;
}