#include "Delay.h"
#include <cmath>

Delay delay;

std::pair<float, float> Delay::GetSamples(const float* recordedSamples)
{
	if (++writePos == audioBuffSize)   { writePos = 0; }
	if (++readPos == audioBuffSize)    { readPos = 0; }
	if (++oldReadPos == audioBuffSize) { oldReadPos = 0;}

	float newSample[4] = {audioBuffer[0][readPos], audioBuffer[1][readPos], audioBuffer[2][readPos], audioBuffer[3][readPos]};

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		float scale = static_cast<float>(delayCrossfade) / static_cast<float>(crossfade);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			float oldSample = audioBuffer[ch][oldReadPos];
			newSample[ch] = static_cast<float>(newSample[ch]) * (1.0f - scale) + static_cast<float>(oldSample) * (scale);
		}
		--delayCrossfade;
	}

	static constexpr float feedbackScale = 1.0f / 4096.0f;
	const float feedback = feedbackScale * adc.delayFeedback;

	// Store the latest recorded sample and the delayed sample back to the audio buffer, shuffling each sample along the stereo field
	audioBuffer[0][writePos] = lpFilter.CalcFilter(recordedSamples[0] + feedback * newSample[3], 0);
	audioBuffer[1][writePos] = lpFilter.CalcFilter(recordedSamples[1] + feedback * newSample[0], 1);
	audioBuffer[2][writePos] = lpFilter.CalcFilter(recordedSamples[2] + feedback * newSample[1], 2);
	audioBuffer[3][writePos] = lpFilter.CalcFilter(recordedSamples[3] + feedback * newSample[2], 3);

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

	// Arrange the delay lines from left to right in the stereo field
	const float leftOut  = FastTanh((0.5 * newSample[0]) + (0.36 * newSample[1]) + (0.15 * newSample[2]));
	const float rightOut = FastTanh((0.5 * newSample[3]) + (0.36 * newSample[2]) + (0.15 * newSample[1]));

	return std::make_pair(leftOut, rightOut);
}


// Algorithm source: https://varietyofsound.wordpress.com/2011/02/14/efficient-tanh-computation-using-lamberts-continued-fraction/
float Delay::FastTanh(const float x)
{
	float x2 = x * x;
	float a = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
	float b = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
	return a / b;
}


void Delay::UpdateFilter()
{
	lpFilter.Update(false);
}
