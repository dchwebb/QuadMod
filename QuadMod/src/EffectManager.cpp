#include "EffectManager.h"
#include "Delay.h"

EffectManager effectManager;

std::pair<float, float> EffectManager::ProcessSamples(Samples& samples)
{
	Samples origSamples = samples;

	if (effect != nullptr) {
		effect->GetSamples(samples);

		// Scale mix from 0 (dry) to  1 (fully wet)
		const float effectMix = adc.effectMix / 4096.0f;
		for (uint32_t i = 0; i < 4; ++i) {
			samples.ch[i] = (1.0f - effectMix) * origSamples.ch[i] + effectMix * samples.ch[i];
		}
	}

	// Stereo mix with effects only
	const float fxOutL = (0.30f * samples.ch[0]) + (0.26f * samples.ch[1]) + (0.24f * samples.ch[2]) + (0.20f * samples.ch[3]);
	const float fxOutR = (0.20f * samples.ch[0]) + (0.24f * samples.ch[1]) + (0.26f * samples.ch[2]) + (0.30f * samples.ch[3]);

	float mixOutL, mixOutR;

	if (delayOn) {
		delay.GetSamples(samples);

		// Arrange the delay lines from left to right in the stereo field
		const float delayOutL  = (0.5f * samples.ch[0]) + (0.36f * samples.ch[1]) + (0.15f * samples.ch[2]);
		const float delayOutR = (0.5f * samples.ch[3]) + (0.36f * samples.ch[2]) + (0.15f * samples.ch[1]);

		const float delayMix = adc.delayMix / 4096.0f;
		mixOutL = (1.0f - delayMix) * fxOutL + delayMix * delayOutL;
		mixOutR = (1.0f - delayMix) * fxOutR + delayMix * delayOutR;

	} else {
		mixOutL = fxOutL;
		mixOutR = fxOutR;
	}

	return std::make_pair(mixOutL, mixOutR);

}



void EffectManager::IdleJobs()
{
	GPIOG->ODR |= GPIO_ODR_OD6;

	delay.IdleJobs();

	GPIOG->ODR &= ~GPIO_ODR_OD6;
}
