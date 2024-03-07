#include "Phaser.h"
#include "Cordic.h"
#include <cmath>

// Code loosely based on:
// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/phaser

Phaser phaser;


void Phaser::GetSamples(Samples& samples)
{
	const uint32_t lfoFreq = (4095 - adc.effectLFOSpeed) * 64;
	uint32_t lfoPhase = lfoInitPhase + lfoFreq;

	const float lfoSweepWidth = static_cast<float>(adc.effectLFORange) * 5.0f;

	static constexpr float feedbackScale = 1.0f / 4096.0f;
	const float feedback = static_cast<float>(4095 - adc.effectRegen) * feedbackScale;


	for (uint32_t channel = 0; channel < 4; ++channel) {

		// Feedback from one channel to the next: last filtered value is output of final filter in bank
		if (feedback != 0.0) {
			samples.ch[channel] += feedback * allpass[(channel + 1) & 3].oldVal[filterCount];
			samples.ch[channel] *= (1.0f - 0.2f * feedback);	// Slightly reduce the level of the sample to avoid distortion
		}

		// Run the sample through each of the all-pass filters
		samples.ch[channel] = FilterSamples(channel, samples.ch[channel], lfoSweepWidth, lfoPhase);

		if (channel == 0) {
			lfoInitPhase = lfoPhase;									// Store phase position of first channel
		} else {
			static constexpr uint32_t phaseDiff = std::pow(2, 30);		// Apply a 45 degree phase shift to each channel
			lfoPhase += phaseDiff;
		}
	}

	// Convert phase to a 12 bit value for LED brightness
	uint32_t brightness = lfoInitPhase >> 19;		// Limit from 0 to 8191
	if (brightness > 4095) {
		brightness = 8192 - brightness;
	}
	lfoLED = brightness;
}


float Phaser::FilterSamples(const uint32_t channel, float sample, const float lfoSweepWidth, const uint32_t phase)
{
	const float freq = (4300.0f - adc.effectLFOBaseFreq) + lfoSweepWidth * (0.5f + 0.5f * Cordic::Sin(phase));
	const float w0 = std::min(freq * inverseSampleRate, 0.99f * pi);
	const float coeff = -Cordic::Tan((0.5f * w0) - (float)M_PI_4);

	for (uint32_t f = 0; f < filterCount; ++f) {
	   // Transfer function is y1 = (b0 * sample) + (b1 * x1) + (a1 * y1) but a1 = b0 and b1 = -1.0 so simplify calculation
		const float out = (coeff * (sample + allpass[channel].oldVal[f + 1])) - allpass[channel].oldVal[f];
		allpass[channel].oldVal[f] = sample;
		sample = out;
	}
	allpass[channel].oldVal[filterCount] = sample;

	return sample;
}



