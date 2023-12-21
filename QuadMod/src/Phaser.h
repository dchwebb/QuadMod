#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>


class Phaser : public Effect {
public:
	void GetSamples(Samples& samples);
	void IdleJobs();

private:
	float FilterSamples(const uint32_t channel, const float sample, const float lfoSweepWidth, const uint32_t phase);

	static constexpr uint32_t filterCount = 4;	// number of allpass filters to use per channel
	struct {
		float coeff;							// Filter co-efficient (simplified)
		float oldVal[filterCount + 1];			// Previous samples
	} allpass[4];

	float baseFrequency = 200.0;				// Lowest frequency of allpass filters
	uint32_t lfoInitPhase = 0;
	uint32_t lfoFreq = 0;						// LFO frequency
};

extern Phaser phaser;




