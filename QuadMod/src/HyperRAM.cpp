#include "HyperRAM.h"

HyperRAM hyperRAM;

uint32_t HyperRAM::Read(uint32_t address, Mode mode)
{
	// ID register 0 at 0x0
	// ID register 1 at 0x2
	// Configuration register 0 at 0x1000
	// Configuration register 1 at 0x1002

	MemMappedOff();
	OCTOSPI1->DLR = (mode == Register ? 1 : 3);				// Return 2 bytes for register access, 4 for memory
	OCTOSPI1->CR = OCTOSPI_CR_FMODE_0;						// 00: Indirect write mode; *01: Indirect read mode; 10: Automatic polling mode; 11: Memory-mapped mode
	OCTOSPI1->CCR |= (OCTOSPI_CCR_ADMODE_2 |				// Address: 000: None; 001: One line; 010: Two lines; 011: Four lines; *100: Eight lines
					  OCTOSPI_CCR_ADSIZE |					// Address size 32 bits
					  OCTOSPI_CCR_ADDTR |					// Address: double data rate
					  OCTOSPI_CCR_DMODE_2 |					// Data: 000: None; 001: One line; 010: Two lines; 011: Four lines; *100: Eight lines
					  OCTOSPI_CCR_DDTR |					// Data: double data rate
					  OCTOSPI_CCR_DQSE);					// DQS Enable

	OCTOSPI1->DCR1 &= ~OCTOSPI_DCR1_MTYP;					// 100: HyperBus memory mode; 101: HyperBus register mode
	OCTOSPI1->DCR1 |= (mode << OCTOSPI_DCR1_MTYP_Pos);		// 100: HyperBus memory mode; 101: HyperBus register mode

	OCTOSPI1->CR |= OCTOSPI_CR_EN;							// Enable OCTOSPI
	OCTOSPI1->AR = address;									// Set address register

	while ((OCTOSPI1->SR & OCTOSPI_SR_TCF) == 0) {};		// Wait until transfer complete
	const uint32_t ret = OCTOSPI1->DR;
	while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {};
	OCTOSPI1->CR &= ~OCTOSPI_CR_EN;							// Disable OCTOSPI
	return ret;
}


uint32_t HyperRAM::Write(uint32_t address, uint32_t val)
{
	MemMappedOff();
	OCTOSPI1->DLR = 0;										// Write 2 bytes
	OCTOSPI1->CR &= ~OCTOSPI_CR_FMODE;						// *00: Indirect write mode; 01: Indirect read mode; 10: Automatic polling mode; 11: Memory-mapped mode
	OCTOSPI1->WCCR |= (OCTOSPI_CCR_ADMODE_2 |				// Address: 000: None; 001: One line; 010: Two lines; 011: Four lines; *100: Eight lines
					  OCTOSPI_CCR_ADSIZE |					// Address size 32 bits
					  OCTOSPI_CCR_ADDTR |					// Address: double data rate
					  OCTOSPI_CCR_DMODE_2 |					// Data: 000: None; 001: One line; 010: Two lines; 011: Four lines; *100: Eight lines
					  OCTOSPI_CCR_DDTR |					// Data: double data rate
					  OCTOSPI_CCR_DQSE);					// DQS Enable
	OCTOSPI1->CCR = OCTOSPI1->WCCR;

	//OCTOSPI1->WTCR |= 4 << OCTOSPI_TCR_DCYC_Pos;

	OCTOSPI1->DCR1 &= ~OCTOSPI_DCR1_MTYP;					// 100: HyperBus memory mode; 101: HyperBus register mode
	OCTOSPI1->DCR1 |= (Memory << OCTOSPI_DCR1_MTYP_Pos);	// 100: HyperBus memory mode; 101: HyperBus register mode

	OCTOSPI1->CR |= OCTOSPI_CR_EN;							// Enable OCTOSPI
	OCTOSPI1->AR = address;									// Set address register
	OCTOSPI1->DR = val;										// Set data register

	while ((OCTOSPI1->SR & OCTOSPI_SR_TCF) == 0) {};		// Wait until transfer complete
	const uint32_t ret = OCTOSPI1->DR;
	while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {};
	OCTOSPI1->CR &= ~OCTOSPI_CR_EN;							// Disable OCTOSPI
	return ret;
}

/*
void HyperRAM::MemoryMapped()
{
	// Activate memory mapped mode
	if (!memMapMode) {
		//SCB_InvalidateDCache_by_Addr(flashAddress, 10000);// Ensure cache is refreshed after write or erase
		while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {};
		CheckBusy();										// Check chip is not still writing data

		OCTOSPI1->ABR = 0xFF;								// Use alternate bytes to pad the address with a dummy byte of 0xFF
		OCTOSPI1->CR |= OCTOSPI_CR_EN;						// Enable QSPI

		OCTOSPI1->CCR = (OCTOSPI_CCR_FMODE |					// 00: Indirect write; 01: Indirect read; 10: Automatic polling; *11: Memory-mapped
						OCTOSPI_CCR_ADSIZE_1 |				// Address: 00: 8-bit ; 01: 16-bit; *10: 24-bit; 11: 32-bit
						OCTOSPI_CCR_ADMODE |				// Address: 00: None; 01: One line; 10: Two lines; *11: Four lines
						OCTOSPI_CCR_ABMODE |				// Alternate Bytes: 00: None; 01: One line; 10: Two lines; *11: Four lines
						OCTOSPI_CCR_DMODE |					// Data: 00: None; 01: One line; 10: Two lines; *11: Four lines
						OCTOSPI_CCR_IMODE_0 |				// Instruction: 00: None; *01: One line; 10: Two lines;
						(4 << OCTOSPI_CCR_DCYC_Pos) |		// insert 4 dummy clock cycles
						(fastReadIO << OCTOSPI_CCR_INSTRUCTION_Pos));

		while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {};
		memMapMode = true;
	}
}
*/

void HyperRAM::MemMappedOff()
{
	if (memMapMode) {
		OCTOSPI1->CR &= ~OCTOSPI_CR_EN;						// Disable OCTOSPI
		OCTOSPI1->CCR = 0;
		while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {};
		memMapMode = false;
	}
}
