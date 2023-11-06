#pragma once

#include "stm32h563xx.h"

extern volatile uint32_t SysTickVal;
static constexpr uint32_t sysTick = 1000;						// 1ms
static constexpr uint32_t sampleRate = 48000;


void InitSystemClock();
void InitSAI();
void InitHardware();
void InitAudioCodec();
void InitSysTick();
void InitMPU();
void InitHyperRAM();

//void InitDAC();
//void InitEnvTimer();
//void InitADC1(volatile uint16_t* buffer, uint16_t channels);
//void InitUart();
//void InitCordic();
//void InitPWMTimer();
//void InitMidiUART();
