#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>

class EffectManager {
public:
	Effect* effect;					// Current effect in use
	bool delayOn = true;

	std::pair<float, float> ProcessSamples(float* samples);
	void IdleJobs();

};

extern EffectManager effectManager;
