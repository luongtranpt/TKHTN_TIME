#ifndef __Temp_H
#define __Temp_H

#include "main.h"

// Ham ngat ngoai lay gia tri tan so
void HAL_GPIO_EXTI_Callback(uint16_t);

// Ham chuyen doi tu gia tri ADC sang nhiet do
double temp_convert(uint16_t);
#endif