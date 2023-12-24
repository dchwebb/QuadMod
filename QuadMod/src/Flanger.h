#pragma once
#include "initialisation.h"
#include "Effect.h"


class Flanger : public Effect {
public:
	void GetSamples(Samples& samples);
	void IdleJobs();

private:
	static constexpr uint32_t audioBuffSize = 4000;
	int32_t writePos = 0;
	Samples audioBuffer[audioBuffSize] = {};

	float baseFrequency = 2.0;				// Lowest frequency of allpass filters
	//float readPos[4] = {baseFrequency, baseFrequency, baseFrequency, baseFrequency};
	uint32_t lfoInitPhase = 0;
	uint32_t lfoFreq = 0;						// LFO frequency
};

extern Flanger flanger;




