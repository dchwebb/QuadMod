int32_t saiTest1 = 0;
uint32_t saiTestInc1 = 100;

uint32_t saiTest2 = 0;
uint32_t saiTest3 = 0;
uint32_t saiTest4 = 0;
uint32_t saiRecAL = 0;
uint32_t saiRecAR = 0;
uint32_t saiRecBL = 0;
uint32_t saiRecBR = 0;
bool saiL = true;
struct {
	uint32_t leftSend;
	uint32_t leftRec;
} debugSAI[256];
uint8_t debugCount = 0;
uint32_t saiCounter = 0;

void SAI1_IRQHandler(void)
{
	// Check if interrupt is Block A FIFO request
	if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0) {
		saiTest1 += saiTestInc1;
		if (saiTest1 >= 65535 || saiTest1 < 0) {
			saiTestInc1 = -saiTestInc1;
			saiTest1 += saiTestInc1;
		}

		saiTest2 += 6000000;
		saiTest3 += 7000000;
		saiTest4 += 8000000;

		SAI1_Block_A->DR = (uint32_t)(saiTest1 << 15);
		SAI1_Block_A->DR = (uint32_t)(saiTest2);
		SAI1_Block_B->DR = (uint32_t)(saiTest3);
		SAI1_Block_B->DR = (uint32_t)(saiTest4);
	}

	// Check if interrupt is data received in SAI2 Block A
	while ((SAI2_Block_A->SR & SAI_xSR_FREQ) != 0) {
		if (saiL) {
			saiRecAL = SAI2_Block_A->DR;
			saiRecBL = SAI2_Block_B->DR;
			debugSAI[debugCount].leftRec = saiRecAL;
			debugSAI[debugCount++].leftSend = saiTest1;
		} else {
			saiRecAR = SAI2_Block_A->DR;
			saiRecBR = SAI2_Block_B->DR;
		}
		saiL = !saiL;
	}

	if (saiCounter == 1) {
		volatile uint32_t susp = 1;
		++susp;
	}
	++saiCounter;
}

void USB_DRD_FS_IRQHandler()
{
	usb.USBInterruptHandler();
}

void SysTick_Handler(void)
{
	++SysTickVal;
}

void NMI_Handler(void)
{
	while (1) {}
}

void HardFault_Handler(void)
{
	while (1) {}
}

void MemManage_Handler(void)
{
	while (1) {}
}

void BusFault_Handler(void)
{
	while (1) {}
}

void UsageFault_Handler(void)
{
	while (1) {}
}




