#include "Phaser.h"
#include "Cordic.h"
#include <numbers>
#include <cmath>

// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/phaser

Phaser phaser;


//__attribute__((optimize("O0")))

void Phaser::GetSamples(Samples& samples)
{
	lfoFrequency = (float)adc.lfoSpeed / 1000.0f;
	lfoSweepWidth = (float)adc.lfoRange * 5.0f;
	feedback = (float)adc.feedback / 4096.0f;
	float lfoPhase = lfoInitPhase + lfoFrequency * inverseSampleRate;

	for (uint32_t channel = 0; channel < 4; ++channel) {

		// Feedback from one channel to the next: last filtered value is output of final filter in bank
		if (feedback != 0.0) {
			samples.ch[channel] += feedback * allpass[(channel + 1) & 3].oldVal[filterCount];
			samples.ch[channel] *= (1.0f - 0.2f * feedback);	// Slightly reduce the level of the sample to avoid distortion
		}

		while (lfoPhase >= 1.0) {				// Ensure phase is between 0 and 1
			lfoPhase -= 1.0;
		}

		// Run the sample through each of the all-pass filters
		samples.ch[channel] = FilterSamples(channel, samples.ch[channel], lfoPhase);

		if (channel == 0) {						// Apply a 45 degree phase shift to each channel
			lfoInitPhase = lfoPhase;
		} else {
			lfoPhase += 0.25;
		}
	}
}


float Phaser::FilterSamples(const uint32_t channel, float sample, const float phase)
{
    // Transfer function is y1 = (b0 * sample) + (b1 * x1) + (a1 * y1) but a1 = b0 and b1 = -1.0 so simplify calculation

	const float freq = baseFrequency + lfoSweepWidth * lfo(phase);
	const float w0 = std::min(freq * inverseSampleRate, 0.99f * pi);
	const float coeff = -Cordic::Tan((0.5f * w0) - (float)M_PI_4);

	for (uint32_t f = 0; f < filterCount; ++f) {
		const float out = (coeff * (sample + allpass[channel].oldVal[f + 1])) - allpass[channel].oldVal[f];
		allpass[channel].oldVal[f] = sample;
		sample = out;
	}
	allpass[channel].oldVal[4] = sample;

	return sample;
}


void Phaser::IdleJobs()
{

}


// Function for calculating LFO waveforms. Phase runs from 0 to 1, output is scaled from 0 to 1
float Phaser::lfo(const float phase)
{
#define LFOSINE
#ifdef LFOSINE
	return 0.5f + 0.5f * Cordic::Sin(2.0f * pi * phase);
#else
	if (phase < 0.25f) {
		return 0.5f + 2.0f * phase;
	} else if (phase < 0.75f) {
		return 1.0f - 2.0f * (phase - 0.25f);
	} else {
		return 2.0f * (phase - 0.75f);
	}
#endif
}

