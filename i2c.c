#include "S32K144.h"
#include "i2c.h"

void LPI2C0_Init(void)
{

    while (SCG->SIRCCSR & SCG_SIRCCSR_LK_MASK);

    // Cấu hình SIRC Range: RANGE=1 cho High Range (8 MHz)
    // RANGE=0: Low Range (2 MHz), RANGE=1: High Range (8 MHz)
    SCG->SIRCCFG = SCG_SIRCCFG_RANGE(1);

    SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV2(1) | SCG_SIRCDIV_SIRCDIV1(1);
    // Enable SIRC
    SCG->SIRCCSR = SCG_SIRCCSR_SIRCEN(1) | SCG_SIRCCSR_SIRCLPEN(1);

    // Chờ SIRC Valid
    while (!(SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK));

    // 1. Cấp clock cho Port A (Nơi chứa chân I2C: PTA2-SDA, PTA3-SCL)
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    // 2. Cấp clock cho LPI2C0
    // Chọn nguồn clock: SIRCDIV2 (8 MHz) -> PCS = 0b010 (2)
    // Enable Clock -> CGC = 1
    PCC->PCCn[PCC_LPI2C0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_PCS(2) | PCC_PCCn_CGC_MASK;

    // MUX = 3 (LPI2C0_SDA/SCL), PE=1, PS=1
    PORTA->PCR[2] |= PORT_PCR_MUX(3) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[3] |= PORT_PCR_MUX(3) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    /* Bước 3: Cấu hình LPI2C Module */
    // 1. Reset và Disable module trước khi cấu hình
    LPI2C0->MCR = LPI2C_MCR_RST_MASK;
    LPI2C0->MCR = 0x00000000;

    // 2. Cấu hình Prescaler (MCFGR1)
    // PRESCALE = 0 (Divide by 1)
    LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);

    LPI2C0->MCCR0 = LPI2C_MCCR0_CLKLO(0x0B) |
                       LPI2C_MCCR0_CLKHI(0x05) |
                       LPI2C_MCCR0_DATAVD(0x02) |
                       LPI2C_MCCR0_SETHOLD(0x04);

    // 3. Bật Module Master (MEN = 1)
    LPI2C0->MCR |= LPI2C_MCR_MEN_MASK;
}


uint8_t LPI2C0_SendFrame(uint8_t slaveAddr, uint8_t *data, uint8_t length)
{
    I2C_State_t state = I2C_START;
    uint32_t timeout;
    uint8_t index = 0;

    while (state != I2C_DONE && state != I2C_ERROR)
    {
        switch (state)
        {
        case I2C_START:
            /* Clear error flags */
            if (LPI2C0->MSR & (LPI2C_MSR_NDF_MASK |
                               LPI2C_MSR_ALF_MASK |
                               LPI2C_MSR_FEF_MASK))
            {
                LPI2C0->MSR = LPI2C_MSR_NDF_MASK |
                              LPI2C_MSR_ALF_MASK |
                              LPI2C_MSR_FEF_MASK |
                              LPI2C_MSR_EPF_MASK;
                LPI2C0->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;
                state = I2C_ERROR;
                break;
            }

            timeout = 100000;
            while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
            if (timeout == 0)
            {
            	state = I2C_ERROR;
            	break;
            }

            /* START + WRITE */
            LPI2C0->MTDR = LPI2C_MTDR_CMD(0x04) |
                           LPI2C_MTDR_DATA(slaveAddr << 1);
            //while(1);
            state = I2C_SEND_DATA;
            break;

        case I2C_SEND_DATA:
            if (index >= length)
            {
                state = I2C_STOP;
                break;
            }

            timeout = 100000;
            while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
            if (timeout == 0)
            {
            	state = I2C_ERROR;
            	break;
            }

            LPI2C0->MTDR = LPI2C_MTDR_CMD(0x00) |
                           LPI2C_MTDR_DATA(data[index++]);
            break;

        case I2C_STOP:
            timeout = 100000;
            while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
            if (timeout == 0)
            {
            	state = I2C_ERROR;
            	break;
            }

            LPI2C0->MTDR = LPI2C_MTDR_CMD(0x02); // STOP

            timeout = 100000;
            while (!(LPI2C0->MSR & LPI2C_MSR_SDF_MASK) && timeout--);
            LPI2C0->MSR = LPI2C_MSR_SDF_MASK;

            state = I2C_DONE;
            break;

        default:
            state = I2C_ERROR;
            break;
        }
    }

    return (state == I2C_DONE);
}

uint8_t LPI2C0_ReadByte(uint8_t slaveAddr, uint8_t *data )
{
	I2C_State_t state = I2C_START;
	uint32_t timeout;
	while (state != I2C_DONE && state != I2C_ERROR)
	{
		switch(state)
		{
		case I2C_START:
			 timeout = 100000;
			 while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
			 if (timeout == 0)
			 {
				 state = I2C_ERROR;
				 break;
			 }
			 LPI2C0->MTDR = LPI2C_MTDR_CMD(0x04) |
             LPI2C_MTDR_DATA((slaveAddr << 1) | 0x01);

			 state = I2C_READ_DATA;
			 break;
		case I2C_READ_DATA:
			timeout = 100000;
		    while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
			if (timeout == 0)
			{
				state = I2C_ERROR;
				break;
			}
			/* RECEIVE 1 byte + NACK */
			LPI2C0->MTDR = LPI2C_MTDR_CMD(0x01);

		    timeout = 100000;
		    while (!(LPI2C0->MSR & LPI2C_MSR_RDF_MASK) && timeout--);
		    if (timeout == 0)
		    {
		    	state = I2C_ERROR;
		    	break;
		    }

		   *data = (uint8_t)(LPI2C0->MRDR & 0xFF);

			state = I2C_STOP;
			break;
		 case I2C_STOP:
		    timeout = 100000;
		    while (!(LPI2C0->MSR & LPI2C_MSR_TDF_MASK) && timeout--);
		    if (timeout == 0)
		    {
		    	state = I2C_ERROR;
		    	break;
		    }

		    LPI2C0->MTDR = LPI2C_MTDR_CMD(0x02);

		    timeout = 100000;
		    while (!(LPI2C0->MSR & LPI2C_MSR_SDF_MASK) && timeout--);
		    LPI2C0->MSR = LPI2C_MSR_SDF_MASK;

		    state = I2C_DONE;
		    break;

		 default:
		    state = I2C_ERROR;
		    break;
		   }
		}

	return (state == I2C_DONE);
}
