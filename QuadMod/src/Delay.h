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

	int16_t delayHysteresis = 900;
	static constexpr int16_t crossfade = 6000;
	static constexpr int16_t tempoHysteresis = 100;

	float lpFilterCutoff = 0.5f;
	Filter<2> lpFilter{filterPass::LowPass, &adc.delayFilter};
};

extern Delay delay;
