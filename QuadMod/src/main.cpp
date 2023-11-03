#include "initialisation.h"
#include "USB.h"
#include "AudioCodec.h"

volatile uint32_t SysTickVal;
volatile uint32_t outputUSB = 0;

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitSystemClock();					// Configure the clock and PLL
	InitHardware();
	audioCodec.Init();
	usb.InitUSB();

	while (1) {
#if (USB_DEBUG)
		if (SysTickVal - outputUSB > 1000 && (GPIOC->IDR & GPIO_IDR_ID13) == GPIO_IDR_ID13) {
			usb.OutputDebug();
			outputUSB = SysTickVal;
		}
#endif
		usb.cdc.ProcessCommand();
	}
}

