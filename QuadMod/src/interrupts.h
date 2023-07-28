uint8_t saiTest = 0;
void SAI1_IRQHandler(void)
{
	SAI1_Block_A->DR = (uint32_t)(++saiTest);
	SAI1_Block_A->DR = (uint32_t)(0x80000000 + saiTest);
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




