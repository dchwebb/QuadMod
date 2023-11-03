#include "AudioCodec.h"
#include "stdio.h"

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
