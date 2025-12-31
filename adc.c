#include "ADC.h"
#include "S32K144.h"
void ADC_init(void)
{
    /* Configure ADC clock source */
    PCC->PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Disable clock to change PCS */
    PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);    /* PCS=1: Select SOSCDIV2 */
    PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;  /* Enable bus clock in ADC */

    /* Disable ADC module initially */
    ADC0->SC1[0] = 0x00001F;                         /* ADCH=1F: Module is disabled for conversions*/
                                                        /* AIEN=0: Interrupts are disabled */

    /* Configure ADC for 12-bit mode */
    ADC0->CFG1 = 0x00000004;                         /* ADICLK=0: Input clk=ALTCLK1=SOSCDIV2 */
                                                        /* ADIV=0: Prescaler=1 */
                                                        /* MODE=1: 12-bit conversion */

    ADC0->CFG2 = 0x0000000C;                         /* SMPLTS=12: sample time is 13 ADC clks */

    ADC0->SC2 = 0x00000000;                          /* ADTRG=0: SW trigger */
                                                        /* ACFE,ACFGT,ACREN=0: Compare func disabled */
                                                        /* DMAEN=0: DMA disabled */
                                                        /* REFSEL=0: Voltage reference pins= VREFH, VREFL */

    ADC0->SC3 = 0x00000000;                          /* CAL=0: Do not start calibration sequence */
                                                        /* ADCO=0: One conversion performed */
                                                        /* AVGE,AVGS=0: HW average function disabled */
}
void convertAdcChan(uint16_t adcChan)
{                                            /* For SW trigger mode, SC1[0] is used */
    ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;   /* Clear prior ADCH bits */
    ADC0->SC1[0] = ADC_SC1_ADCH(adcChan); /* Initiate Conversion*/
}
uint8_t adc_complete(void)
{
    return ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT); /* Wait for completion */
}
uint32_t read_adc_chx(void)
{
    uint32_t adc_result = 0;
    adc_result = ADC0->R[0]; /* For SW trigger mode, R[0] is used */
    return adc_result;
}
