#include "Delay.h"
#include <cmath>

Delay delay;

void Delay::GetSamples(Samples& samples)
{
	if (++writePos == audioBuffSize)   { writePos = 0; }
	if (++readPos == audioBuffSize)    { readPos = 0; }
	if (++oldReadPos == audioBuffSize) { oldReadPos = 0;}

	Samples newSample = {audioBuffer[0][readPos], audioBuffer[1][readPos], audioBuffer[2][readPos], audioBuffer[3][readPos]};

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		float scale = static_cast<float>(delayCrossfade) / static_cast<float>(crossfade);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			float oldSample = audioBuffer[ch][oldReadPos];
			newSample.ch[ch] = static_cast<float>(newSample.ch[ch]) * (1.0f - scale) + static_cast<float>(oldSample) * (scale);
		}
		--delayCrossfade;
	}

	static constexpr float feedbackScale = 1.0f / 4096.0f;
	const float feedback = feedbackScale * adc.delayFeedback;

	// Store the latest recorded sample and the delayed sample back to the audio buffer, shuffling each sample along the stereo field
	audioBuffer[0][writePos] = lpFilter.CalcFilter(samples.ch[0] + feedback * newSample.ch[3], 0);
	audioBuffer[1][writePos] = lpFilter.CalcFilter(samples.ch[1] + feedback * newSample.ch[0], 1);
	audioBuffer[2][writePos] = lpFilter.CalcFilter(samples.ch[2] + feedback * newSample.ch[1], 2);
	audioBuffer[3][writePos] = lpFilter.CalcFilter(samples.ch[3] + feedback * newSample.ch[2], 3);

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

//	// Arrange the delay lines from left to right in the stereo field
//	const float leftOut  = (0.5 * newSample[0]) + (0.36 * newSample[1]) + (0.15 * newSample[2]);
//	const float rightOut = (0.5 * newSample[3]) + (0.36 * newSample[2]) + (0.15 * newSample[1]);
//
//	return std::make_pair(leftOut, rightOut);
}



void Delay::IdleJobs()
{
	lpFilter.Update(false);
}
