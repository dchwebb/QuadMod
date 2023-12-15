#include "PassThrough.h"


PassThrough passThrough;

std::pair<float, float> PassThrough::GetSamples(const float* recordedSamples)
{
	const float leftOut  = (0.5 * recordedSamples[2]) + (0.36 * recordedSamples[1]) + (0.15 * recordedSamples[0]);
	const float rightOut = (0.5 * recordedSamples[3]) + (0.36 * recordedSamples[0]) + (0.15 * recordedSamples[1]);

	return std::make_pair(leftOut, rightOut);
}


void PassThrough::IdleJobs()
{
}



