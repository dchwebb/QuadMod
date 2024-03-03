#pragma once
#include "initialisation.h"
#include "AudioCodec.h"
#include "Effect.h"
#include <tuple>

class EffectManager {
public:
	Effect* effect;					// Current effect in use
	static constexpr uint32_t audioBuffSize = 4000;
	Samples audioBuffer[audioBuffSize] = {};

	bool delayOn = false;

	std::pair<float, float> ProcessSamples(Samples& samples);
	void IdleJobs();
private:
	float EqualPowerCrossfade(const float mix, const float sample1, const float sample2);
};

extern EffectManager effectManager;
