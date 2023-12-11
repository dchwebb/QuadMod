#pragma once
#include "initialisation.h"
#include <tuple>

// Struct added to classes to enable them to process audio codec samples
class Effect {
public:
	virtual std::pair<float, float> GetSamples(const float* samples) { return std::make_pair(0.0f, 0.0f); };
};
