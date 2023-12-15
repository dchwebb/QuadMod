#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>


class PassThrough : public Effect {
public:
	std::pair<float, float> GetSamples(const float* recordedSamples);
	void IdleJobs();

private:

};

extern PassThrough passThrough;




