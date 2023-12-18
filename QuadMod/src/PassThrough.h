#pragma once
#include "initialisation.h"
#include "Effect.h"
#include <tuple>


class PassThrough : public Effect {
public:
	void GetSamples(float* recordedSamples);
	void IdleJobs();

private:

};

extern PassThrough passThrough;




