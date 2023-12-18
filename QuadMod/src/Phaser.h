#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>


class Phaser : public Effect {
public:
	void GetSamples(float* samples);
	void IdleJobs();

private:
	float lfo(const float phase);
	float FilterSample(const uint32_t channel, const float sample, const uint32_t filter);
	float FilterSamples(const uint32_t channel, const float sample, const float phase);
	void UpdateCoefficients(const uint32_t channel);

	static constexpr uint32_t filterCount = 4;	// number of allpass filters to use per channel
	struct {
		float coeff;			// Filter co-efficient (simplified)
		float oldVal[filterCount + 1];			// Previous samples
	} allpass[4];

	float baseFrequency = 200.0;		// Lowest frequency of allpass filters

	float depth= 1.0;					// Mix level for phase-shifted signal (0-1)
	float feedback = 0.0;				// Feedback level for feedback phaser (0-<1)
	float lfoInitPhase = 0.0f;
	float lfoFrequency = 0.5;			// LFO frequency (Hz)
	float lfoSweepWidth  = 750.0;			// Amount of change from min to max delay
	//float filterCutoff[1000];
	float filterCutoff;
};

extern Phaser phaser;




