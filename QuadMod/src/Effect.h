#pragma once
#include "initialisation.h"
#include <tuple>

struct Samples {
	float ch[4];
};

// Struct added to classes to enable them to process 4 audio codec samples and return stereo output
class Effect {
public:
	virtual void GetSamples(Samples& samples);
	void IdleJobs() {};
};
