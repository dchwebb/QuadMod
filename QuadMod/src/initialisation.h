#pragma once

#include "stm32h563xx.h"

extern volatile uint32_t SysTickVal;
static constexpr uint32_t sysTick = 1000;						// 1ms
static constexpr uint32_t sampleRate = 48000;

struct ADCValues {
	uint16_t val1;
	uint16_t val2;
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

//void InitDAC();
//void InitEnvTimer();
//void InitUart();
//void InitCordic();
//void InitPWMTimer();
//void InitMidiUART();
