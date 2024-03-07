#include "AudioCodec.h"
#include "EffectManager.h"
#include <stdio.h>
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
	pdnPin.SetHigh();											// Set PDN high once SPI CS is high to enable SPI mode
	DelayMS(10);												// Wait 10ms for codec power to stablise

	SendCmd({Command::activateSPI, Command::SpiMode, 0x7A});	// To activate SPI mode send 0xDE 0xADDA 0x7A
	WriteData(Command::AudioInterfaceFormat, 0b0000'1100);		// Set 32bit, data on falling bit clock
	WriteData(Command::AnalogInput,          0b1111'1111);		// Set all in channels to pseudo differential input mode
	WriteData(Command::AnalogFilter,         0b0010'0010);		// Use a fast filter for ADC - reduces latency by ~300uS

	InitSAI();													// Configure I2S via SAI peripheral and start clocks
	WriteData(Command::PowerManagement,      0b0011'0111);		// Release standby state
}


void AudioCodec::WriteData(uint16_t address, uint8_t data)
{
	SendCmd({Command::Write, address, data});
}


uint8_t AudioCodec::ReadData(uint16_t address)
{
	while ((SPI1->SR & SPI_SR_RXP) == SPI_SR_RXP) {
		[[maybe_unused]] volatile uint32_t dummy = SPI1->RXDR;	// Clear read buffer
	}
	SendCmd({Command::Read, address, 0});

	return (uint8_t)(SPI1->RXDR & 0xFF);						// Will also echo back command - data is last byte
}


inline void AudioCodec::SendCmd(Command cmd)
{
	SPI1->IFCR |= (SPI_IFCR_TXTFC | SPI_IFCR_EOTC);				// Clear status register

	SPI1->TXDR = *(uint32_t*)&cmd;
	SPI1->CR1 |= SPI_CR1_CSTART;

	while ((SPI1->SR & SPI_SR_EOT) == 0);						// Wait for command to finish
}


// For generating test signals
float sinePos = 0.0f;
int32_t sinOutput = 0;
int32_t triInc = 10000000;
int32_t triOutput = 0;

void AudioCodec::Interrupt()
{
	debugPin.SetHigh();

	// Input: SAI2 Block A FIFO request
	while ((SAI2_Block_A->SR & SAI_xSR_FREQ) != 0) {
		if (leftRight) {
			dataIn.ch[0] = normalise32Bit * (int32_t)SAI2_Block_A->DR;
			dataIn.ch[2] = normalise32Bit * (int32_t)SAI2_Block_B->DR;
		} else {
			dataIn.ch[1] = normalise32Bit * (int32_t)SAI2_Block_A->DR;
			dataIn.ch[3] = normalise32Bit * (int32_t)SAI2_Block_B->DR;
		}
		leftRight = !leftRight;
	}

	/*
	// Generate test signals for output and loop-back testing
	sinePos += 0.01f;
	if (sinePos > 2.0f * M_PI) {
		sinePos -= 2.0f * M_PI;
	}
	sinOutput = (int32_t)(std::sin(sinePos) * std::numeric_limits<int32_t>::max());

	int32_t newTri = triOutput + triInc;
	if ((triInc > 0 && newTri < triOutput) || (triInc < 0 && newTri > triOutput)) {
		triInc = -triInc;
	} else {
		triOutput = newTri;
	}


	SAI1_Block_A->DR = Denormalise(dataIn.ch[0]);
	SAI1_Block_A->DR = Denormalise(dataIn.ch[3]);
	SAI1_Block_B->DR = std::bit_cast<uint32_t>(sinOutput);
	SAI1_Block_B->DR = std::bit_cast<uint32_t>(triOutput);
*/


	// Output: SAI1 Block A FIFO request
	if ((SAI1_Block_A->SR & SAI_xSR_FREQ) != 0) {
		auto [left, right] = effectManager.ProcessSamples(dataIn);

		// Main stereo outputs are Block A; Block B outputs are debug only
		SAI1_Block_A->DR = Denormalise(left);
		SAI1_Block_B->DR = Denormalise(left);
		SAI1_Block_A->DR = Denormalise(right);
		SAI1_Block_B->DR = Denormalise(right);

		outputDone = true;			// Tell the main loop it can run idle jobs
	}

	debugPin.SetLow();
}


int32_t AudioCodec::Denormalise(const float x)
{
	// Apply FastTan approximation to limit sample from -1 to +1
	float x2 = x * x;
	float a = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
	float b = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
	return (int32_t)(denormalise32Bit * (a / b));
}

