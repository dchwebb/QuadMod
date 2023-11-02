#include "AudioCodec.h"

AudioCodec audioCodec;

void AudioCodec::Init()
{
	InitAudioCodec();

	// Wait 10ms for codec power to stablise
	uint32_t powerEnabled = SysTickVal;
	while (SysTickVal < powerEnabled + 10);

	Command_t cmd (Command_t::Write, Command_t::AnalogInput, 0b11111111);	// Set all channels to pseudo differential input mode
	SendCmd(cmd);
}

void AudioCodec::SendCmd(Command_t cmd)
{
	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;
}


