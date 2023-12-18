#include "EffectManager.h"
#include "Delay.h"

EffectManager effectManager;

std::pair<float, float> EffectManager::ProcessSamples(float* samples)
{
	effect->GetSamples(samples);
	if (delayOn) {
		delay.GetSamples(samples);
	}

	// Arrange the delay lines from left to right in the stereo field
	const float leftOut  = (0.5 * samples[0]) + (0.36 * samples[1]) + (0.15 * samples[2]);
	const float rightOut = (0.5 * samples[3]) + (0.36 * samples[2]) + (0.15 * samples[1]);
	return std::make_pair(leftOut, rightOut);

}



void EffectManager::IdleJobs()
{
	GPIOG->ODR |= GPIO_ODR_OD6;

	delay.IdleJobs();

	GPIOG->ODR &= ~GPIO_ODR_OD6;
}
