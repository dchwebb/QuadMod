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
	InitAudioCodec();

	DelayMS(2);
	GPIOD->ODR |= GPIO_ODR_OD15;					// Set output high with SPI CS high to enable SPI mode
	DelayMS(10);									// Wait 10ms for codec power to stablise

	// To activate SPI send 0xDE 0xADDA 0x7A
	SendCmd({Command::activateSPI, Command::SpiMode, 0x7A});

	while ((SPI1->SR & SPI_SR_EOT) == 0);			// Wait for command to finish

	uint8_t data = ReadData(Command::AudioInterfaceFormat);
	printf("Data: %02X\r\n", data);

	//WriteData(Command::AnalogInput, 0b11111111);	// Set all channels to pseudo differential input mode


}

void AudioCodec::WriteData(uint16_t address, uint8_t data)
{
	SendCmd({Command::Write, address, data});
}


uint8_t AudioCodec::ReadData(uint16_t address)
{
	volatile uint32_t dummy = SPI1->RXDR;
	SendCmd({Command::Read, address, 0});

	while ((SPI1->SR & SPI_SR_EOT) == 0);
	return (uint8_t)(SPI1->RXDR & 0xFF);				// Will also echo back command
}


void AudioCodec::SendCmd(Command cmd)
{
	SPI1->IFCR |= (SPI_IFCR_TXTFC | SPI_IFCR_EOTC);		// Clear status register

	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;
}
