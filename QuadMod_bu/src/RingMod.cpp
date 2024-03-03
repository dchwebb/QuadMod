#include "RingMod.h"
#include "EffectManager.h"
#include <cmath>

RingMod ringMod;

inline float SampleFromOffset(uint32_t writePos, uint32_t offset, uint32_t channel)
{
	int32_t readPos = (int32_t)writePos - offset;
	if (readPos < 0) {
		readPos += effectManager.audioBuffSize;
	}
	return effectManager.audioBuffer[readPos].ch[channel];
}


void RingMod::GetSamples(Samples& samples)
{
	for (uint32_t channel = 0; channel < 4; ++channel) {
		effectManager.audioBuffer[writePos].ch[channel] = lpFilter.FilterSample(samples.ch[channel], channel);
		//effectManager.audioBuffer[writePos].ch[channel] = samples.ch[channel];

		// Calculate read position - only increments every other sample to generate octave down
		if (writePos & 1) {
			if (++readOffset[channel] == effectManager.audioBuffSize) {
				readOffset[channel] = 0;
			}
		}

		// Check to see if write sample is at a zero crossing
		if (effectManager.audioBuffer[writePos].ch[channel] > 0 && lessThanZero[write][channel]) {
			crossingOffset[channel] = 0;
			lessThanZero[write][channel] = false;
		} else if (effectManager.audioBuffer[writePos].ch[channel] < 0 && !lessThanZero[write][channel]) {
			lessThanZero[write][channel] = true;
		}

		// Get current read sample
		float readSample = SampleFromOffset(writePos, readOffset[channel], channel);

		// Check to see if read sample is at a zero crossing
		if (readOffset[channel] > minCycle) {
			if (readSample > 0 && lessThanZero[read][channel]) {
				readOffset[channel] = crossingOffset[channel];
				lessThanZero[read][channel] = false;
				readSample = SampleFromOffset(writePos, readOffset[channel], channel);
			} else if (readSample < 0 && !lessThanZero[read][channel]) {
				lessThanZero[read][channel] = true;
			}
		}

		// Increment Write crossing offset
		if (++crossingOffset[channel] == effectManager.audioBuffSize) {
			crossingOffset[channel] = 0;
		}

		if (octaveOnly) {
			samples.ch[channel] = lpFilter.FilterSample(readSample, channel + 4);
		} else {
			samples.ch[channel] *= lpFilter.FilterSample(readSample, channel + 4);
		}
		//samples.ch[channel] = readSample;
	}

	if (++writePos == effectManager.audioBuffSize) {
		writePos = 0;
	}
}



void RingMod::IdleJobs()
{

}


