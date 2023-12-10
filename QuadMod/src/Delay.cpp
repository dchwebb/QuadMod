#include "Delay.h"
#include <cmath>

Delay delay;

std::pair<float, float> Delay::GetSamples(float* recordedSamples)
{
	if (++writePos == audioBuffSize) {
		writePos = 0;
	}
	if (++readPos == audioBuffSize) {
		readPos = 0;
	}
	if (++oldReadPos == audioBuffSize) {
		oldReadPos = 0;
	}
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


	float leftOut  = (0.75 * newSample[0]) + (0.5 * newSample[1]) + (0.25 * newSample[2]);
	float rightOut = (0.75 * newSample[3]) + (0.5 * newSample[2]) + (0.25 * newSample[1]);

	audioBuffer[0][writePos] = recordedSamples[0] + 0.75 * newSample[3];
	audioBuffer[1][writePos] = recordedSamples[1] + 0.75 * newSample[0];
	audioBuffer[2][writePos] = recordedSamples[2] + 0.75 * newSample[1];
	audioBuffer[3][writePos] = recordedSamples[3] + 0.75 * newSample[2];


	calcDelay = std::min((uint32_t)(1000 + adc.delayTime) * 6, audioBuffSize);

	// If delay time has changed trigger crossfade from old to new read position
//	if (delayCrossfade > 0) {
//		--delayCrossfade;
//	}
	if (delayCrossfade == 0 && std::abs(calcDelay - currentDelay) > delayHysteresis) {
		oldReadPos = readPos;
		readPos = writePos - calcDelay - 1;
		while (readPos < 0) {
			readPos += audioBuffSize;
		}
		delayCrossfade = crossfade;
		currentDelay = calcDelay;
	}


	return std::make_pair(leftOut, rightOut);
}
