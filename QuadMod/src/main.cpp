#include "initialisation.h"

volatile uint32_t SysTickVal;


extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitSystemClock();					// Configure the clock and PLL
	SystemCoreClockUpdate();			// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitSAI();

	while (1) {
	}
}

