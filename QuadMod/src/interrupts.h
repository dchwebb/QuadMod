uint16_t saiTest = 0;
uint32_t saiRecBL = 0;
uint32_t saiRecBR = 0;
bool saiL = true;

void SAI1_IRQHandler(void)
{
	// Check if interrupt is Block A FIFO request
	if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0) {
		SAI1_Block_A->DR = (uint32_t)(++saiTest);
		SAI1_Block_A->DR = (uint32_t)(0x80000000 + saiTest);
	}

	// Check if interrupt is data received in Block B
	if ((SAI1_Block_B->SR & SAI_xSR_FREQ) != 0) {
		if (saiL) {
			saiRecBL = SAI1_Block_B->DR;
		} else {
			saiRecBR = SAI1_Block_B->DR;
		}
		saiL = !saiL;
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




