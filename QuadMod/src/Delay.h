#pragma once

#include "initialisation.h"
#include "Filter.h"
#include "EFfect.h"
#include <tuple>

class Delay {
public:
	void GetSamples(Samples& samples);
	void IdleJobs();

private:
	void CheckForClock();

	static constexpr uint32_t audioBuffSize = 34000;
	int32_t writePos = 0;
	int32_t readPos = 1000;
	Samples audioBuffer[audioBuffSize] = {};

	int32_t oldReadPos;
	uint16_t delayCrossfade;			// Counter that ticks down during a crossfade following delay length change
	int32_t currentDelay = 0;			// Used to trigger crossfade from old to new read position
	int32_t calcDelay = 0;				// Delay time according to whether clocked and with multipliers applied
	int16_t hysteresisPotVal;			// For hysteresis checking
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 6, 8};
	//float delayMult = 1.0f;				// Multipliers for delay in clocked mode
	volatile int32_t clockAdjust = -8;

	float delayTimePot = 0.0f;			// Smoothed value
	float delayTimeCV = 0.0f;

	int16_t delayHysteresis = 900;
	static constexpr int16_t crossfade = 6000;
	static constexpr int16_t tempoHysteresis = 50;

	float lpFilterCutoff = 0.5f;
	Filter<2> lpFilter{filterPass::LowPass, &adc.delayFilter, true};

	uint32_t delayCounter;				// Counter used to calculate clock times in sample time
	uint32_t lastClock;					// Time last clock signal received in sample time
	uint32_t clockInterval;				// Clock interval in sample time
	bool clockValid = false;
	bool clockHigh = false;

	// Testing for whether clock or LFO is being used as delay time CV
	uint32_t nonClockVals = 0;			// Counter to store ADC values that are neither valid clock high or low
	uint32_t nonClockTime = 0;			// Last SysTick time a sequence of non-clock values were on ADC
	uint32_t clockTest = 1000;			// Interval inside which intermediate signals on ADC invalidate clock

	uint32_t ledCounter = 0;
	uint32_t ledBrightness = 0;
	volatile uint32_t& clockedDelayLED = TIM4->CCR1;
	volatile uint32_t& unclockedDelayLED = TIM4->CCR3;

	GpioPin clockInPin{GPIOC, 12, GpioPin::Type::Input};	// PC12 - clock in

};

extern Delay delay;
