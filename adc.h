#ifndef ADC_H_
#define ADC_H_

#include "stdint.h"
#include "S32K144.h"

void ADC_init(void);
void convertAdcChan(uint16_t adcChan);
uint8_t adc_complete(void);
uint32_t read_adc_chx(void);

#endif
