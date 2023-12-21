#include "Delay.h"
#include <cmath>

Delay delay;

void Delay::GetSamples(Samples& samples)
{
	if (++writePos == audioBuffSize)   { writePos = 0; }
	if (++readPos == audioBuffSize)    { readPos = 0; }
	if (++oldReadPos == audioBuffSize) { oldReadPos = 0;}

	Samples newSample = audioBuffer[readPos];

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		float scale = static_cast<float>(delayCrossfade) / static_cast<float>(crossfade);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			float oldSample = audioBuffer[oldReadPos].ch[ch];
			newSample.ch[ch] = static_cast<float>(newSample.ch[ch]) * (1.0f - scale) + static_cast<float>(oldSample) * (scale);
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
//	audioBuffer[writePos].ch[1] = lpFilter.CalcFilter(samples.ch[1] + feedback * newSample.ch[0], 1);
//	audioBuffer[writePos].ch[2] = lpFilter.CalcFilter(samples.ch[2] + feedback * newSample.ch[1], 2);
//	audioBuffer[writePos].ch[3] = lpFilter.CalcFilter(samples.ch[3] + feedback * newSample.ch[2], 3);

	calcDelay = std::min((uint32_t)(1000 + adc.delayTime) * 6, audioBuffSize);

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
