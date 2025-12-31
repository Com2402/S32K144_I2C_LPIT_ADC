/*
 * clocks_and_modes.h
 *
 * Description: Header file for clock and mode configuration functions
 */

#ifndef CLOCKS_AND_MODES_H_
#define CLOCKS_AND_MODES_H_

#include "S32K144.h"

/*
 * Function: SOSC_init_8MHz
 * Description: Initializes the System Oscillator (SOSC) with 8 MHz configuration
 * Parameters: None
 * Return: void
 */
void SOSC_init_8MHz(void);

/*
 * Function: SPLL_init_160MHz
 * Description: Initializes the System PLL (SPLL) to generate 160 MHz clock
 * Parameters: None
 * Return: void
 */
void SPLL_init_160MHz(void);

/*
 * Function: NormalRUNmode_80MHz
 * Description: Configures the device to Normal RUN mode with 80 MHz core clock
 *              Uses PLL as clock source with appropriate dividers for core, bus, and slow clocks
 * Parameters: None
 * Return: void
 */
void NormalRUNmode_80MHz(void);

#endif /* CLOCKS_AND_MODES_H_ */
