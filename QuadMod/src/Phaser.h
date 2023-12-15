#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>





class Phaser : public Effect {
public:
	std::pair<float, float> GetSamples(const float* recordedSamples);
	void IdleJobs();

private:
	float lfo(const float phase);
	float FilterSample(const uint32_t channel, const float sample, const uint32_t filter);
	float FilterSamples(const uint32_t channel, const float sample);
	void UpdateCoefficients(const uint32_t channel);

	static constexpr uint32_t filterCount = 4;	// number of allpass filters to use per channel
	struct {
		float phase = 0.0f;
		float coeff;			// Filter co-efficient (simplified)

		struct {
			float x1;
			float y1;
		} filters[filterCount + 1];			// Previous samples


		float oldVal[filterCount + 1];			// Previous samples

	} allpass[4];

	float baseFrequency = 200.0;		// Lowest frequency of allpass filters
	float sweepWidth  = 2000.0;			// Amount of change from min to max delay

	float depth= 1.0;					// Mix level for phase-shifted signal (0-1)
	float feedback = 0.0;				// Feedback level for feedback phaser (0-<1)
	float lfoFrequency = 0.5;			// LFO frequency (Hz)

	uint32_t refreshChannel= 0;			// Store channel of the next batch of filters to updated in idle jobs

	float lastFilterOutputs[4];			// Store last output sample for use in feedback loop
};

extern Phaser phaser;




