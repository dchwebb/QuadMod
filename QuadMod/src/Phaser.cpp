#pragma GCC push_options
#pragma GCC optimize ("O1")

#include "Phaser.h"
#include "Cordic.h"
#include <numbers>
#include <cmath>


// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/phaser

Phaser phaser;

uint8_t debugRollover = 0;
uint32_t debugCnt = 0;

//__attribute__((optimize("O0")))

std::pair<float, float> Phaser::GetSamples(const float* recordedSamples)
{
	lfoFrequency = (float)adc.lfoSpeed / 1000.0f;
	lfoSweepWidth = (float)adc.lfoRange * 10.0f;
	feedback = (float)adc.feedback / 4096.0f;
	float lfoPhase = lfoInitPhase + lfoFrequency * inverseSampleRate;

	float out[4] = {recordedSamples[0], recordedSamples[1], recordedSamples[2], recordedSamples[3]};

	for (uint32_t channel = 0; channel < 4; ++channel) {

		// If feedback is enabled, include the feedback from the last sample in the
		// input of the allpass filter chain. This is actually not accurate to how
		// analog phasers work because there is a sample of delay between output and
		// input, which adds a further phase shift of up to 180 degrees at half the
		// sampling frequency. To truly model an analog phaser with feedback involves
		// modelling a delay-free loop, which is beyond the scope of this example.

		// last filtered value is output of final filter in bank
		if (feedback != 0.0) {
			out[channel] += feedback * allpass[(channel + 1) & 3].oldVal[filterCount];
		}

		while (lfoPhase >= 1.0) {
			lfoPhase -= 1.0;										// Ensure phase is between 0 and 1
		}

		// Run the sample through each of the all-pass filters
		out[channel] = FilterSamples(channel, out[channel], lfoPhase);

		// Add all-pass signal to the output (depth = 0: input only; depth = 1: evenly balanced input and output
		out[channel] = (1.0f - 0.5f * depth) * recordedSamples[channel] + 0.5f * depth * out[channel];

		if (channel == 0) {
			lfoInitPhase = lfoPhase;
		} else {
			lfoPhase += 0.25;					// Apply a 45 degree phase shift to each channel
		}

	}

	const float leftOut  = (0.3 * out[0]) + (0.2 * out[1]) + (0.36 * out[2]) + (0.14 * out[3]);
	const float rightOut = (0.2 * out[0]) + (0.3 * out[1]) + (0.14 * out[2]) + (0.36 * out[3]);

	return std::make_pair(leftOut, rightOut);
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

	if (channel == 0) {
		filterCutoff = w0;
//		if (++debugRollover == 0 && debugCnt < 1000) {
//			filterCutoff[debugCnt++] = w0;
//		}
	}
	return sample;
}


void Phaser::UpdateCoefficients(const uint32_t channel)
{
//	// This code based on calculations by Julius O. Smith:
//	// https://ccrma.stanford.edu/~jos/pasp/Classic_Virtual_Analog_Phase.html
//
//	// Avoid passing pi/2 to the tan function...
//	//const float freq = baseFrequency + sweepWidth * lfo(allpass[channel].phase);
//	const float freq = baseFrequency * std::pow(sweepWidth, lfo(allpass[channel].phase));
//	const float w0 = std::min(freq * inverseSampleRate, 0.99f * std::numbers::pi_v<float>);
//
//	// Only store one coefficient as a1 = b0 and b1 = 1.0
//	allpass[channel].coeff = -std::tan((0.5f * w0) - (std::numbers::pi_v<float> / 4));

}


void Phaser::IdleJobs()
{
	GPIOG->ODR |= GPIO_ODR_OD6;


	GPIOG->ODR &= ~GPIO_ODR_OD6;
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

#pragma GCC pop_options
