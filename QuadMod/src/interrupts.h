uint16_t saiTest = 0;
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

void SAI1_IRQHandler(void)
{
	// Check if interrupt is Block A FIFO request
	if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0) {
		SAI1_Block_A->DR = (uint32_t)(++saiTest);
		SAI1_Block_A->DR = (uint32_t)(0x10000000 + saiTest);
		SAI1_Block_B->DR = (uint32_t)(0x20000000 + saiTest);
		SAI1_Block_B->DR = (uint32_t)(0x30000000 + saiTest);
	}

	// Check if interrupt is data received in SAI2 Block A
	if ((SAI2_Block_A->SR & SAI_xSR_FREQ) != 0) {
		saiRecAL = SAI2_Block_A->DR;
		saiRecAR = SAI2_Block_A->DR;
		saiRecBL = SAI2_Block_B->DR;
		saiRecBR = SAI2_Block_B->DR;

		debugSAI[debugCount].leftRec = saiRecAL;
		debugSAI[debugCount++].leftSend = saiTest;
	} else {
		uint32_t susp = 1;
		++susp;
	}
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




