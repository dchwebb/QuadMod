#pragma once
#include "initialisation.h"
#include "AudioCodec.h"
#include "Effect.h"
#include <tuple>

class EffectManager {
public:
	Effect* effect;					// Current effect in use
	bool delayOn = false;

	std::pair<float, float> ProcessSamples(Samples& samples);
	void IdleJobs();

};

extern EffectManager effectManager;
