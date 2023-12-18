#pragma once
#include "initialisation.h"
#include <tuple>

// Struct added to classes to enable them to process 4 audio codec samples and return stereo output
class Effect {
public:
	virtual void GetSamples(float* samples);
	void IdleJobs() {};
};
