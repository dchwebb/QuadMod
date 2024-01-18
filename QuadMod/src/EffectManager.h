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

	bool delayOn = true;

	std::pair<float, float> ProcessSamples(Samples& samples);
	void IdleJobs();

};

extern EffectManager effectManager;
