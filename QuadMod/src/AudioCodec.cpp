#include "AudioCodec.h"
#include "stdio.h"
#include <numbers>
#include <cmath>
#include <bit>

AudioCodec audioCodec;

static inline void DelayMS(uint32_t ms)
{
	uint32_t delay = SysTickVal;
	while (SysTickVal < delay + ms);
}

void AudioCodec::Init()
{
	InitAudioCodec();											// Initialise SPI and PDN pin
	DelayMS(2);													// Wait for SPI CS pin to go high
	GPIOD->ODR |= GPIO_ODR_OD15;								// Set PDN high once SPI CS is high to enable SPI mode
	DelayMS(10);												// Wait 10ms for codec power to stablise

	SendCmd({Command::activateSPI, Command::SpiMode, 0x7A});	// To activate SPI mode send 0xDE 0xADDA 0x7A

//	uint8_t data = ReadData(Command::AudioInterfaceFormat);
//	printf("Data: %02X\r\n", data);

	WriteData(Command::AudioInterfaceFormat, 0b00001110);		// Set 32bit, data on rising bit clock
	WriteData(Command::AnalogInput, 0b11111111);				// Set all channels to pseudo differential input mode

	InitSAI();													// Configure I2S via SAI peripheral and start clocks
	WriteData(Command::PowerManagement, 0b00110111);			// Release standby state
}

void AudioCodec::WriteData(uint16_t address, uint8_t data)
{
	SendCmd({Command::Write, address, data});
}


uint8_t AudioCodec::ReadData(uint16_t address)
{
	[[maybe_unused]] volatile uint32_t dummy = SPI1->RXDR;	// Clear read buffer
	SendCmd({Command::Read, address, 0});

//	while ((SPI1->SR & SPI_SR_EOT) == 0);
	return (uint8_t)(SPI1->RXDR & 0xFF);					// Will also echo back command - data is last byte
}


inline void AudioCodec::SendCmd(Command cmd)
{
	SPI1->IFCR |= (SPI_IFCR_TXTFC | SPI_IFCR_EOTC);			// Clear status register

	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;

	while ((SPI1->SR & SPI_SR_EOT) == 0);			// Wait for command to finish
}



float sinePos = 0.0f;
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
int32_t output = 0;

void AudioCodec::TestOutput()
{
	// Check if interrupt is Block A FIFO request
	if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0) {
		GPIOG->ODR |= GPIO_ODR_OD12;
		sinePos += 0.01f;
		output = (int32_t)(std::sin(sinePos) * std::numeric_limits<int32_t>::max());

		saiTest1 += saiTestInc1;
		if (saiTest1 >= 65535 || saiTest1 < 0) {
			saiTestInc1 = -saiTestInc1;
			saiTest1 += saiTestInc1;
		}

		saiTest2 += 6000000;
		saiTest3 += 7000000;
		saiTest4 += 8000000;

		//SAI1_Block_A->DR = (uint32_t)(saiTest1 << 15);
		SAI1_Block_A->DR = std::bit_cast<uint32_t>(output);

		SAI1_Block_A->DR = (uint32_t)(saiTest2);
		SAI1_Block_B->DR = (uint32_t)(saiTest3);
		SAI1_Block_B->DR = (uint32_t)(saiTest4);
		GPIOG->ODR &= ~GPIO_ODR_OD12;
	}

	// Check if interrupt is data received in SAI2 Block A
	while ((SAI2_Block_A->SR & SAI_xSR_FREQ) != 0) {
		GPIOG->ODR |= GPIO_ODR_OD6;
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
		GPIOG->ODR &= ~GPIO_ODR_OD6;
	}

	if (saiCounter == 1) {
		volatile uint32_t susp = 1;
		++susp;
	}
	++saiCounter;
}
