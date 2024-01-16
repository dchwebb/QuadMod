#include "Flanger.h"
#include "EffectManager.h"
#include "Cordic.h"
#include <cmath>

// Code loosely based on:
// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/flanger/Source

Flanger flanger;

float debugOffset[1000];
uint32_t debugPos;

void Flanger::GetSamples(Samples& samples)
{
	const uint32_t lfoFreq = adc.lfoSpeed * 128;
	uint32_t lfoPhase = lfoInitPhase + lfoFreq;
	lfoInitPhase = lfoPhase;

	// lfoSweepWidth needs to be between baseFrequency and audioBuffSize
	const float lfoSweepWidth = baseFrequency + (static_cast<float>(adc.lfoRange)  / 4096.0f) * 100.0f;

	static constexpr float feedbackScale = 1.0f / 4096.0f;
	const float feedback = static_cast<float>(adc.feedback) * feedbackScale;

	static constexpr float freqScale = 50.0f / 4096.0f;
	baseFrequency = adc.baseFreq * freqScale;

	if (++writePos == effectManager.audioBuffSize) {
		writePos = 0;
	}

	for (uint32_t channel = 0; channel < 4; ++channel) {

		// Calculate read position
		const float readOffset = (baseFrequency + lfoSweepWidth * (0.5f + 0.5f * Cordic::Sin(lfoPhase)));
		volatile float readPos = writePos + readOffset;
		while (readPos >= static_cast<float>(effectManager.audioBuffSize)) {
			readPos -= effectManager.audioBuffSize;
		}

		if (channel == 0) {
			debugOffset[debugPos++] = readPos;
			if (debugPos == 1000) {
				debugPos = 0;
			}
		}

		// Interpolate two samples
		volatile const uint32_t floor = std::floor(readPos);
		uint32_t ceil = std::ceil(readPos);
		if (ceil >= static_cast<float>(effectManager.audioBuffSize)) {
			ceil -= effectManager.audioBuffSize;
		}

		const float flangeSample = std::lerp(effectManager.audioBuffer[floor].ch[channel], effectManager.audioBuffer[ceil].ch[channel], readPos - floor);

		effectManager.audioBuffer[writePos].ch[channel] = samples.ch[channel] + feedback * flangeSample;


		samples.ch[channel] = flangeSample;

		if (channel > 0) {
			static constexpr uint32_t phaseDiff = std::pow(2, 30);		// Apply a 45 degree phase shift to each channel
			lfoPhase += phaseDiff;
		}
	}
}



void Flanger::IdleJobs()
{

}

