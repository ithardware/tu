#ifndef __HuoYan_H
#define __HuoYan_H
#include "stm32f10x_conf.h"
#define ADC1_DR_Address   ((uint32_t)0x4001244C)

#define temp_value     Adc1_temp[0]

#define motor_adc_value1   Adc1_temp[3]
#define motor_adc_value2   Adc1_temp[2]
#define motor_adc_value3   Adc1_temp[1]



extern  __IO uint16_t Adc1_temp[5];
extern  void ADC1_Init(void);
extern  void ADC_DMA(void);

#endif
