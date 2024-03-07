#pragma once
#include "initialisation.h"
#include "Effect.h"


class Flanger : public Effect {
public:
	void GetSamples(Samples& samples);
	bool wide = false;
private:
	float SampleFromReadOffset(const float readOffset, const uint32_t channel);

	int32_t writePos = 0;
	uint32_t lfoInitPhase = 0;

	volatile uint32_t& lfoLED = TIM4->CCR2;
};

extern Flanger flanger;




