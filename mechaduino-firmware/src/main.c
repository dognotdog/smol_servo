
#include "atmel_start.h"



/* Arduino Bootloader Clock Setup:

enable external XOSC32K, XTAL

GCLK(1) is setup to do no division, use XOSC32K, enable
GLCK(0) is setup to to use GLCK(1)

DFLL is setup to be ONDEMAND=0, 31 coarse, 511 fine max steps, 1464 mul
DFLL enable wait lock and quick lock disable, ONDEMAND=0

Switch GLCK(0) to use DFLLM48 as source with Improve-Duty-Cycle (IDC) on

*/


int main(void)
{
	// reset chip to clock CPU from ULP32K after bootloader
	// at this point GCLK0 is running off DFLL48M, GCLK1 is running off the external XOSC32K
	// thus:

	// // 1. switch GCLK1 to OSCULP32K

 //    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1);
	// while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}; // GLCK sync
 //    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_GENEN;
	// while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}; // GLCK sync

	// // 2. switch GLCK0 to GLCK1


	// // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0u) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_CLKCTRL_CLKEN | GCLK_GENCTRL_GENEN;
	// // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_CLKCTRL_CLKEN | GCLK_GENCTRL_GENEN;
	// // GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
	// GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	// while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}; // GLCK sync


	// 1. enable 8Mhz internal OSC
	// SYSCTRL->OSC8M.reg = SYSCTRL_OSC8M_PRESC(0);
	// SYSCTRL->OSC8M.bit.ENABLE = 1;
 //    do {} while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY) == 0);

 //    // 2. switch GLCK0 to OSC8M
 //    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0);
	// while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}; // GLCK sync
 //    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
	// while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}; // GLCK sync

	// // 3. disable DFLL
	// SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
 //    // while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0) {};

 //    // 4. enable XOSC32K
 //    SYSCTRL->XOSC32K.reg =  SYSCTRL_XOSC32K_STARTUP(6) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
 //    SYSCTRL->XOSC32K.bit.ENABLE = 1;
 //    do {} while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0);


	atmel_start_init();

	bool ledOn = true;
	while (1) 
	{
		gpio_set_pin_level(MECH_LED, ledOn);
		ledOn = !ledOn;
		delay_ms(200);
	}

}