#pragma once
#include "initialisation.h"
#include <tuple>

// Struct added to classes to enable them to process 4 audio codec samples and return stereo output
class Effect {
public:
	virtual std::pair<float, float> GetSamples(const float* samples) { return std::make_pair(0.0f, 0.0f); };
	virtual void IdleJobs() {};
};
