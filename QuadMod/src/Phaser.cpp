#include "Phaser.h"
#include "Cordic.h"
#include <numbers>
#include <cmath>


// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/phaser

Phaser phaser;

uint8_t debugRollover = 0;
uint32_t debugCnt = 0;

//__attribute__((optimize("O0")))

void Phaser::GetSamples(float* samples)
{
	lfoFrequency = (float)adc.lfoSpeed / 1000.0f;
	lfoSweepWidth = (float)adc.lfoRange * 10.0f;
	feedback = (float)adc.feedback / 4096.0f;
	float lfoPhase = lfoInitPhase + lfoFrequency * inverseSampleRate;

	float origSamples[4] = {samples[0], samples[1], samples[2], samples[3]};

	for (uint32_t channel = 0; channel < 4; ++channel) {

		// last filtered value is output of final filter in bank
		if (feedback != 0.0) {
			samples[channel] += feedback * allpass[(channel + 1) & 3].oldVal[filterCount];
		}

		while (lfoPhase >= 1.0) {
			lfoPhase -= 1.0;										// Ensure phase is between 0 and 1
		}

		// Run the sample through each of the all-pass filters
		samples[channel] = FilterSamples(channel, samples[channel], lfoPhase);

		// Add all-pass signal to the output (depth = 0: input only; depth = 1: evenly balanced input and output
		samples[channel] = (1.0f - 0.5f * depth) * origSamples[channel] + 0.5f * depth * samples[channel];

		if (channel == 0) {
			lfoInitPhase = lfoPhase;
		} else {
			lfoPhase += 0.25;					// Apply a 45 degree phase shift to each channel
		}

	}

}


float Phaser::FilterSamples(const uint32_t channel, float sample, const float phase)
{
    // Transfer function is y1 = (b0 * sample) + (b1 * x1) + (a1 * y1) but a1 = b0 and b1 = -1.0 so simplify calculation

	const float freq = baseFrequency + lfoSweepWidth * lfo(phase);

	//const float freq = baseFrequency * std::pow(sweepWidth, lfo(phase));
	const float w0 = std::min(freq * inverseSampleRate, 0.99f * pi);
	//const float coeff = -std::tan((0.5f * w0) - (float)M_PI_4);
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
float __attribute__((optimize("O1"))) Phaser::lfo(const float phase)
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

