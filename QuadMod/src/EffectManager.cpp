#include "EffectManager.h"
#include "Delay.h"
#include "Flanger.h"
#include "Phaser.h"
#include <cstring>

EffectManager effectManager;

float EffectManager::EqualPowerCrossfade(const float mix1, const float sample1, const float sample2)
{
	// See https://signalsmith-audio.co.uk/writing/2021/cheap-energy-crossfade/
	const float mix2 = 1.0f - mix1;
	const float A = mix1 * mix2;
	const float B = A * (1.0f + 1.4186f * A);
	const float C = B + mix1;
	const float D = B + mix2;
	return (sample1 * C * C) + (sample2 * D * D);
}


std::pair<float, float> EffectManager::ProcessSamples(Samples& samples)
{
	Samples origSamples = samples;

	EffectType();
	effect->GetSamples(samples);

	// Scale mix from 0 (dry) to  1 (fully wet)
	const float effectMix = adc.effectMix / 4096.0f;
	for (uint32_t i = 0; i < 4; ++i) {
		samples.ch[i] = EqualPowerCrossfade(effectMix, samples.ch[i], origSamples.ch[i]);
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
		mixOutL = EqualPowerCrossfade(delayMix, delayOutL, fxOutL);
		mixOutR = EqualPowerCrossfade(delayMix, delayOutR, fxOutR);

	} else {
		mixOutL = fxOutL;
		mixOutR = fxOutR;
	}

	return std::make_pair(mixOutL, mixOutR);

}


void EffectManager::EffectType()
{
	// Handle effect switching (Phaser = pin high; flanger = pin low)
	if (effect == nullptr || fxTypePin.IsHigh() != (effect == &phaser)) {
		std::memset(audioBuffer, 0, audioBuffSize * sizeof(Samples));			// Clear sample buffer
		effect = fxTypePin.IsHigh() ? (Effect*)&phaser : (Effect*)&flanger;
	}
}



void EffectManager::IdleJobs()
{
	//GPIOG->ODR |= GPIO_ODR_OD6;

	delay.IdleJobs();

	//GPIOG->ODR &= ~GPIO_ODR_OD6;
}
