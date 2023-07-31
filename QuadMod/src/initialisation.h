#pragma once

#include "stm32h563xx.h"

extern volatile uint32_t SysTickVal;
#define SYSTICK 1000						// 1ms

void InitSystemClock();
void InitSAI();
void InitHardware();
void InitSysTick();
//void InitDAC();
//void InitIO();
//void InitEnvTimer();
//void InitADC1(volatile uint16_t* buffer, uint16_t channels);
//void InitADC3(volatile uint16_t* buffer, uint16_t channels);
//void InitADC4(volatile uint16_t* buffer, uint16_t channels);
//void InitUart();
//void InitCordic();
//void InitPWMTimer();
//void InitMidiUART();
//void InitSPI2();
//void InitSPI1();
//void InitTunerTimer();
