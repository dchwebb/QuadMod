#pragma once
#include "initialisation.h"
#include "Effect.h"
#include "Filter.h"

class RingMod : public Effect {
public:
	void GetSamples(Samples& samples);
	void IdleJobs();

private:
	enum {read, write};

	uint32_t writePos = 0;
	uint32_t readOffset[4] = {};
	uint32_t cycle[4] = {};
	bool lessThanZero[2][4] = {};
	uint32_t crossingOffset[4] = {};
	static constexpr uint32_t startCycle = 100;
	static constexpr uint32_t minCycle = 20;

	float lpFilterCutoff = 0.5f;
	FixedFilter<2, 4> lpFilter{filterPass::LowPass, 0.1f};
};

extern RingMod ringMod;




