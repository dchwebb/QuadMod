#include "AudioCodec.h"
#include "stdio.h"

AudioCodec audioCodec;

void AudioCodec::Init()
{
	InitAudioCodec();

	// Wait 10ms for codec power to stablise
	uint32_t powerEnabled = SysTickVal;
	while (SysTickVal < powerEnabled + 10);

	//WriteData(Command::AnalogInput, 0b11111111);	// Set all channels to pseudo differential input mode
	uint8_t data = ReadData(Command::AudioInterfaceFormat);
	printf("Data: %02X\r\n", data);
}

void AudioCodec::WriteData(uint16_t address, uint8_t data)
{
	Command cmd (Command::Write, address, data);
	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;
}


uint8_t AudioCodec::ReadData(uint16_t address)
{
	Command cmd (Command::Read, address, 0);
	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;

	while ((SPI1->SR & SPI_SR_RXP) == 0);
	return (uint8_t)SPI1->RXDR;
}
