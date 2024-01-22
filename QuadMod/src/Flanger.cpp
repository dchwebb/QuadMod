#include "Flanger.h"
#include "EffectManager.h"
#include "Cordic.h"
#include <cmath>

// Code loosely based on:
// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/flanger/Source

Flanger flanger;

float Flanger::SampleFromReadOffset(const float readOffset, const uint32_t channel)
{
	volatile float readPos = writePos - readOffset;
	while (readPos <= -1.0f) {
		readPos += effectManager.audioBuffSize;
	}

	// Interpolate two samples
	volatile int32_t floor = std::floor(readPos);

	const float fractionalPos = readPos - floor;				// Handle situation where sample position is between -1 and 0
	if (floor < 0) {
		floor += effectManager.audioBuffSize;
	}

	int32_t ceil = std::ceil(readPos);
	if (ceil >= static_cast<float>(effectManager.audioBuffSize)) {
		ceil -= effectManager.audioBuffSize;
	}

	return std::lerp(effectManager.audioBuffer[floor].ch[channel], effectManager.audioBuffer[ceil].ch[channel], fractionalPos);

}


void Flanger::GetSamples(Samples& samples)
{
	const uint32_t lfoFreq = adc.lfoSpeed * 128;
	uint32_t lfoPhase = lfoInitPhase + lfoFreq;
	lfoInitPhase = lfoPhase;

	// needs to be between baseFrequency and audioBuffSize
	const float lfoSweepWidth = (adc.lfoRange / 4096.0f) * 100.0f;
	const float feedback = adc.feedback / 4096.0f;
	const float baseFreq = (adc.baseFreq / 4096.0f) * 500.0f;

	if (++writePos == effectManager.audioBuffSize) {
		writePos = 0;
	}
	Samples wideFlange;

	for (uint32_t channel = 0; channel < 4; ++channel) {

		// Calculate read position
		const float lfoPos = (0.5f + 0.5f * Cordic::Sin(lfoPhase));		// LFO sine wave scaled from 0 to 1
		const float readOffset = 2.0f + baseFreq + (lfoSweepWidth * lfoPos);
		const float flangeSample = SampleFromReadOffset(readOffset, channel);

		if (wide) {
			const float readOffset2 = 2.0f + baseFreq + (lfoSweepWidth * (1.0f - lfoPos));
			const float flangeSample2 = SampleFromReadOffset(readOffset2, channel);
			wideFlange.ch[channel] = flangeSample2;
		}

		effectManager.audioBuffer[writePos].ch[channel] = samples.ch[channel] + feedback * flangeSample;

		samples.ch[channel] = flangeSample;

		if (channel > 0) {
			static constexpr uint32_t phaseDiff = std::pow(2, 30);		// Apply a 45 degree phase shift to each channel
			lfoPhase += phaseDiff;
		}
	}

	// Add in some stereo sample
	if (wide) {
		for (uint32_t channel = 0; channel < 4; ++channel) {
			samples.ch[channel] += wideFlange.ch[(channel + 2) & 3] * 0.5f;
		}
	}

}



void Flanger::IdleJobs()
{

}


