#pragma once

#include "initialisation.h"
#include "Filter.h"
#include "EFfect.h"
#include <tuple>

class Delay : public Effect {
public:
	// functions inherited from Effect for sample generation and filter recalculation
	void GetSamples(Samples& samples);
	void IdleJobs();

private:
	static constexpr uint32_t audioBuffSize = 34000;
	int32_t writePos = 0;
	int32_t readPos = 1000;
	Samples audioBuffer[audioBuffSize] = {};

	int32_t oldReadPos;
	uint16_t delayCrossfade;			// Counter that ticks down during a crossfade following delay length change
	int32_t currentDelay = 0;			// Used to trigger crossfade from old to new read position
	int32_t calcDelay = 0;				// Delay time according to whether clocked and with multipliers applied
	int16_t delayPotVal;				// For hysteresis checking
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 6, 8};
	float delayMult = 1.0f;				// Multipliers for delay in clocked mode
	volatile int32_t clockAdjust = -8;

	int16_t delayHysteresis = 900;
	static constexpr int16_t crossfade = 6000;
	static constexpr int16_t tempoHysteresis = 50;

	float lpFilterCutoff = 0.5f;
	Filter<2> lpFilter{filterPass::LowPass, &adc.delayFilter};

	uint32_t delayCounter;				// Counter used to calculate clock times in sample time
	uint32_t lastClock;					// Time last clock signal received in sample time
	uint32_t clockInterval;				// Clock interval in sample time
	bool clockValid = false;
	bool clockHigh = false;

	uint32_t ledCounter = 0;
	volatile uint32_t& unclockedDelayLED = TIM4->CCR2;
	volatile uint32_t& clockedDelayLED = TIM4->CCR3;

	GpioPin clockInPin{GPIOC, 12, GpioPin::Type::Input};	// PC12 - clock in

};

extern Delay delay;
