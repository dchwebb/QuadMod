#pragma once
#include "initialisation.h"
#include "Effect.h"


class Flanger : public Effect {
public:
	void GetSamples(Samples& samples);
	void IdleJobs();
	bool wide = true;
private:
	float SampleFromReadOffset(const float readOffset, const uint32_t channel);

	int32_t writePos = 0;
	//float baseFrequency = 2.0;				// Lowest frequency of allpass filters
	uint32_t lfoInitPhase = 0;
};

extern Flanger flanger;




