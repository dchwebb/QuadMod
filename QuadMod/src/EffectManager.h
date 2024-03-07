#pragma once
#include "initialisation.h"
#include "AudioCodec.h"
#include "Effect.h"
#include <tuple>

class EffectManager {
public:
	Effect* effect;					// Current effect in use
	bool delayOn = true;

	static constexpr uint32_t audioBuffSize = 4000;
	Samples audioBuffer[audioBuffSize] = {};

	std::pair<float, float> ProcessSamples(Samples& samples);
	void EffectType();
	void IdleJobs();
private:
	float EqualPowerCrossfade(const float mix, const float sample1, const float sample2);

	GpioPin fxTypePin{GPIOD, 6, GpioPin::Type::Input};	// PD6: Effect Type
};

extern EffectManager effectManager;
