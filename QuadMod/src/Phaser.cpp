#pragma GCC push_options
#pragma GCC optimize ("O1")

#include "Phaser.h"
#include <numbers>
#include <cmath>

Phaser phaser;

std::pair<float, float> Phaser::GetSamples(const float* recordedSamples)
{
	//float channel0EndPhase = lfoPhase;

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
			out[channel] += feedback * allpass[channel].filters[filterCount - 1].y1;
		}

		// Run the sample through each of the all-pass filters
		for (uint32_t f = 0; f < filterCount; ++f) {
			out[channel] = FilterSample(channel, out[channel], f);
		}

		// Add all-pass signal to the output (depth = 0: input only; depth = 1: evenly balanced input and output
		out[channel] = (1.0f - 0.5f * depth) * recordedSamples[channel] + 0.5f * depth * out[channel];

		// Update the LFO phase
		float ph = allpass[0].phase + (channel * 0.125);	// Apply a 45 degree phase shift to each channel
		ph += lfoFrequency * inverseSampleRate;				// Increment the phase by the LFO frequency
		while (ph >= 1.0) {
			ph -= 1.0;										// Ensure phase is between 0 and 1
		}

		// Update the filter frequency for idle jobs calculation
		allpass[channel].phase = ph;
	}

	const float leftOut  = (0.5 * out[2]) + (0.36 * out[1]) + (0.15 * out[0]);
	const float rightOut = (0.5 * out[3]) + (0.36 * out[0]) + (0.15 * out[1]);

	return std::make_pair(leftOut, rightOut);
}


void Phaser::IdleJobs()
{
	GPIOG->ODR |= GPIO_ODR_OD6;

	// Sequentially update the filters for each channel during the idle time between sample processing
	UpdateCoefficients(refreshChannel);
	if (++refreshChannel > 3) {
		refreshChannel = 0;
	}

	GPIOG->ODR &= ~GPIO_ODR_OD6;
}


// Function for calculating LFO waveforms. Phase runs from 0-1, output is scaled from 0 to 1
float __attribute__((optimize("O1"))) Phaser::lfo(const float phase)
{
	if (phase < 0.25f) {
		return 0.5f + 2.0f * phase;
	} else if (phase < 0.75f) {
		return 1.0f - 2.0f * (phase - 0.25f);
	} else {
		return 2.0f * (phase - 0.75f);
	}
}


float __attribute__((optimize("O1"))) Phaser::FilterSample(const uint32_t channel, const float sample, const uint32_t f)
{
    // Transfer function is (b0 * sample) + (b1 * x1) + (a1 * y1) but a1 = b0 and b1 = -1.0 so simplify calculation
	// filters[f].y1 = (b0 * sample) + (b1 * filters[f].x1) + (a1 * filters[f].y1);
	allpass[channel].filters[f].y1 = (allpass[channel].coeff * (sample + allpass[channel].filters[f].y1)) - allpass[channel].filters[f].x1;
	allpass[channel].filters[f].x1 = sample;
    return allpass[channel].filters[f].y1;
}


void __attribute__((optimize("O1"))) Phaser::UpdateCoefficients(const uint32_t channel)
{
	// This code based on calculations by Julius O. Smith:
	// https://ccrma.stanford.edu/~jos/pasp/Classic_Virtual_Analog_Phase.html

	// Avoid passing pi/2 to the tan function...
	//const float freq = baseFrequency + sweepWidth * lfo(allpass[channel].phase);
	const float freq = baseFrequency * std::pow(sweepWidth, lfo(allpass[channel].phase));
	const float w0 = std::min(freq * inverseSampleRate, 0.99f * std::numbers::pi_v<float>);

	// Only store one coefficient as a1 = b0 and b1 = 1.0
	allpass[channel].coeff = -std::tan((0.5f * w0) - (std::numbers::pi_v<float> / 4));

}

#pragma GCC pop_options
