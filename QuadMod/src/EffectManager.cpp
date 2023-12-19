#include "EffectManager.h"
#include "Delay.h"

EffectManager effectManager;

std::pair<float, float> EffectManager::ProcessSamples(Samples& samples)
{
	effect->GetSamples(samples);
	if (delayOn) {
		delay.GetSamples(samples);
	}

	// Arrange the delay lines from left to right in the stereo field
	const float leftOut  = (0.5 * samples.ch[0]) + (0.36 * samples.ch[1]) + (0.15 * samples.ch[2]);
	const float rightOut = (0.5 * samples.ch[3]) + (0.36 * samples.ch[2]) + (0.15 * samples.ch[1]);
	return std::make_pair(leftOut, rightOut);

}



void EffectManager::IdleJobs()
{
	GPIOG->ODR |= GPIO_ODR_OD6;

	delay.IdleJobs();

	GPIOG->ODR &= ~GPIO_ODR_OD6;
}
