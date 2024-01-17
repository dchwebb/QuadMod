#pragma once
#include "initialisation.h"
#include "Effect.h"


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
};

extern RingMod ringMod;




