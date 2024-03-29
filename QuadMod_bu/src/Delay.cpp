#include "Delay.h"
#include <cmath>

Delay delay;

void Delay::GetSamples(Samples& samples)
{
	// Check if clock received
	if ((GPIOA->IDR & GPIO_IDR_ID6) != GPIO_IDR_ID6) {
		if (!clockHigh) {
			clockInterval = delayCounter - lastClock - clockAdjust;			// FIXME constant found by trial and error - probably relates to filtering group delay
			lastClock = delayCounter;
			clockHigh = true;
		}
	} else {
		clockHigh = false;
	}
	clockValid = (delayCounter - lastClock < (sampleRate * 2));					// Valid clock interval is within a second
	++delayCounter;

	if (++writePos == audioBuffSize)   { writePos = 0; }
	if (++readPos == audioBuffSize)    { readPos = 0; }
	if (++oldReadPos == audioBuffSize) { oldReadPos = 0;}

	Samples newSample = audioBuffer[readPos];

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		const float scale = static_cast<float>(delayCrossfade) / static_cast<float>(crossfade);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			const float oldSample = audioBuffer[oldReadPos].ch[ch];
			newSample.ch[ch] = newSample.ch[ch] * (1.0f - scale) + oldSample * scale;
		}
		--delayCrossfade;
	}

	static constexpr float feedbackScale = 1.0f / 4096.0f;
	const float feedback = feedbackScale * adc.delayFeedback;

	// Store the latest recorded sample and the delayed sample back to the audio buffer, shuffling each sample along the stereo field
	for (uint32_t ch = 0; ch < 4; ++ch) {
		uint32_t newCh = (ch + 3) & 3;

		audioBuffer[writePos].ch[ch] = lpFilter.CalcFilter(samples.ch[ch] + feedback * newSample.ch[newCh], ch);
	}

	// Calculate delay times - either clocked or not

	if (clockValid) {
		if (abs(delayPotVal - adc.delayTime) > tempoHysteresis) {
			delayPotVal = adc.delayTime;										// Store value for hysteresis checking
			delayMult = tempoMult[tempoMult.size() * adc.delayTime / 4096];		// get tempo multiplier from lookup
			calcDelay = delayMult * (clockInterval / 2);
			while (calcDelay > audioBuffSize) {
				calcDelay /= 2;
			}
		}
	} else {
		calcDelay = std::min((uint32_t)(1000 + adc.delayTime) * 6, audioBuffSize);
	}

	// If delay time has changed trigger crossfade from old to new read position
	if (delayCrossfade == 0 && std::abs(calcDelay - currentDelay) > delayHysteresis) {
		oldReadPos = readPos;
		readPos = writePos - calcDelay - 1;
		while (readPos < 0) {
			readPos += audioBuffSize;
		}
		delayCrossfade = crossfade;
		currentDelay = calcDelay;
	}

	samples = newSample;
}



void Delay::IdleJobs()
{
	lpFilter.Update(false);
}
