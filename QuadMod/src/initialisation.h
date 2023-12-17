#pragma once

#include "stm32h563xx.h"

extern volatile uint32_t SysTickVal;
static constexpr uint32_t sysTick = 1000;						// 1ms
static constexpr uint32_t sampleRate = 48000;
static constexpr float inverseSampleRate  = 1.0f / (float)sampleRate;

union ADCValues {
	struct {
		uint16_t delayFeedback;
		uint16_t delayTime;
		uint16_t delayFilter;
	};
	struct {
		uint16_t lfoSpeed;
		uint16_t lfoRange;
		uint16_t feedback;
	};
};



struct DMALinkedList {
	uint32_t TR1;
	uint32_t TR2;
	uint32_t BR1;
	uint32_t SAR;
	uint32_t DAR;
	uint32_t LLR;
};

extern volatile ADCValues adc;

void InitSystemClock();
void InitSAI();
void InitHardware();
void InitAudioCodec();
void InitSysTick();
void InitMPU();
void InitHyperRAM();
void InitADC2(volatile uint16_t* buffer, uint16_t channels);
void InitDAC();
void InitCordic();
