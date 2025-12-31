/* Copyright 2023 NXP */
/* License: BSD 3-clause
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
#include "S32K144.h"
#include "clock_and_modes.h"
#include "adc.h"
#include "i2c.h"

#if defined(__ghs__)
#define __INTERRUPT_SVC __interrupt
#define __NO_RETURN _Pragma("ghs nowarning 111")
#elif defined(__ICCARM__)
#define __INTERRUPT_SVC __svc
#define __NO_RETURN _Pragma("diag_suppress=Pe111")
#elif defined(__GNUC__)
#define __INTERRUPT_SVC __attribute__((interrupt("SVC")))
#define __NO_RETURN
#else
#define __INTERRUPT_SVC
#define __NO_RETURN
#endif
#define LED0            0U
#define LED_RED         15U
#define LED_GREEN       16U
#define LED_GPIO_PORT   PTD
int lpit0_ch0_flag_counter = 0;

volatile uint8_t current_adc_channel = 12;   // channel đang dùng
volatile uint8_t next_adc_channel = 12;
volatile uint8_t i2c_read_request = 0;
volatile uint8_t i2c_cmd_from_slave = 0;


typedef struct
{
    uint16_t adc_average;
    uint16_t count_value;
} Framedata_t;

Framedata_t frame_data;

#define S32_NVIC_BASE (0xE000E100u)
#define S32_NVIC ((S32_NVIC_Type *)S32_NVIC_BASE)

void LED_init(void)
{
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    PORTD->PCR[LED_RED]   = PORT_PCR_MUX(1);
    PORTD->PCR[LED_GREEN] = PORT_PCR_MUX(1);
    //PORTD->PCR[LED0]      = PORT_PCR_MUX(1);

    PTD->PDDR |= (1U << LED_RED) | (1U << LED_GREEN);

    /* Tắt LED ban đầu  */
    PTD->PCOR = (1U << LED_RED) | (1U << LED_GREEN);
}

void GPIO_init(void)
{

    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Configure PTB0 as GPIO output (pin_1 for ADC timing) */
    PORTB->PCR[0] = PORT_PCR_MUX(1); /* MUX=1: GPIO */
    PTB->PDDR |= (1U << 0);          /* Set as output */
    PTB->PCOR = (1U << 0);

    /* Configure PTB1 as GPIO output */
    PORTB->PCR[1] = PORT_PCR_MUX(1); /* MUX=1: GPIO */
    PTB->PDDR |= (1U << 1);          /* Set as output */
    PTB->PCOR = (1U << 1);           /* Initial state: LOW */
}

void NVIC_init_IRQs(void)
{
    /* IRQ48-LPIT0 ch0: Clear any pending IRQ */
    S32_NVIC->ICPR[1] = 1U << (48U % 32U);

    /* IRQ48-LPIT0 ch0: Enable IRQ */
    S32_NVIC->ISER[1] = 1U << (48U % 32U);

    /* Set priority to 10 */
    S32_NVIC->IP[48] = (10U << 4U);
}

void LPIT0_init(void)
{

    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs */
    LPIT0->MCR = 0x00000001;
    LPIT0->MIER = 0x00000001;     /* TIE0=1: Timer Interrupt Enabled Chan 0 */

    /* Timeout period: 100ms */
    LPIT0->TMR[0].TVAL = 4000000; /* Chan 0 Timeout: 100ms 40MHz */
    LPIT0->TMR[0].TCTRL = 0x00000001;

}

void LPIT0_Ch0_IRQHandler(void)
{
    uint32_t dummy;
    uint8_t cmd;

    LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
    dummy = LPIT0->MSR;               /* Read-after-write to ensure flag clears before ISR exit */
    (void)dummy;

    lpit0_ch0_flag_counter++;            /* Increment LPIT0 timeout counter */
    current_adc_channel = next_adc_channel;

    /* Toggle pin_1 HIGH - Start ADC processing */
    PTB->PSOR = (1U << 0);
    //PTD->PSOR = (1U << LED_RED);
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++)
    {
        convertAdcChan(current_adc_channel);
        uint32_t timeout = 10000;
        while (adc_complete() == 0 && timeout > 0)
        {
            timeout--;
        };
        if (timeout > 0)
        {
            sum += read_adc_chx();
        }
    }

    frame_data.adc_average = (uint16_t)(sum / 8);
    frame_data.count_value = (uint16_t)lpit0_ch0_flag_counter;

    /* Toggle pin_1 LOW - ADC done */
    PTB->PCOR = (1U << 0);
    //PTD->PSOR = (1U << LED_RED);

    /* Kiểm tra và xóa lỗi I2C trước khi gửi */
    if (LPI2C0->MSR & (LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK | LPI2C_MSR_FEF_MASK))
    {
        LPI2C0->MSR = LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK | LPI2C_MSR_FEF_MASK | LPI2C_MSR_EPF_MASK;
        LPI2C0->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK; /* Reset FIFO */
    }

    /* Toggle pin_2 HIGH - Start I2C  */
    PTB->PSOR = (1U << 1);
    //PTD->PSOR = (1U << LED_GREEN);

    LPI2C0_SendFrame(0x68, (uint8_t *)&frame_data, (uint8_t)sizeof(frame_data));
//    if(LPI2C0_ReadByte(0x68, &cmd))
//    {
//           i2c_cmd_from_slave = cmd;
//           if(cmd != 0x00)
//           {
//               next_adc_channel = cmd;
//           }
//    }


    /* Toggle pin_2 LOW - I2C transmission done */
    PTB->PCOR = (1U << 1);
    //PTD->PSOR = (1U << LED_GREEN);
    /*I2C READ*/
    if(LPI2C0_ReadByte(0x68, &cmd))
    {
          i2c_cmd_from_slave = cmd;
          if(cmd != 0x00)
          {
             next_adc_channel = cmd;
          }
    }
}

int main(void)
{
    /*  clocks and modes */
    SOSC_init_8MHz();           /* Initialize System Oscillator 8 MHz */
    SPLL_init_160MHz();         /* Initialize SPLL to 160 MHz */
    NormalRUNmode_80MHz();      /* Switch to Normal RUN mode: Core 80 MHz, Bus 40 MHz */

    GPIO_init();
    ADC_init();
    LPI2C0_Init();
    LPIT0_init();
    NVIC_init_IRQs();

    for (;;)
    {
//    	if(i2c_read_request){
//    		uint8_t cmd;
//    		i2c_read_request = 0;
//    		if(LPI2C0_ReadByte(0x32, &cmd)){
//    		    i2c_cmd_from_slave = cmd;
//    			if(cmd != 0x00){
//    				next_adc_channel = cmd;
//    			}
//    		}
//    	}
        /* All work is done in interrupt handler */
    }

    return 0;
}
