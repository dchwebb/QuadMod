#include "Delay.h"
#include <cmath>

Delay delay;

uint32_t nonClockVals = 0;		// Counter to store ADC values that are neither valid clock high or low
uint32_t nonClockTime = 0;		// Last SysTick time a sequence of non-clock values were on ADC
uint32_t clockTest = 1000;		// Interval inside which intermediate signals on ADC invalidate clock

void Delay::CheckForClock()
{
	// Check if intermediate values on the adc look like a non-clock value is present
	if (adc.delayTimeCV > 1000 && adc.delayTimeCV < 3500) {
		if (++nonClockVals > 20) {
			nonClockVals = 0;
			nonClockTime = SysTickVal;
		}
	} else {
		nonClockVals = 0;
	}

	const bool nonClock = (SysTickVal - nonClockTime < clockTest);				// Check that sufficient time has elapsed sin ce last non-clock signal

	if (clockInPin.IsHigh()) {
		if (!clockHigh) {
			clockInterval = delayCounter - lastClock - clockAdjust;			// FIXME constant found by trial and error - probably relates to filtering group delay
			lastClock = delayCounter;
			clockHigh = true;
		}
	} else {
		clockHigh = false;
	}
	clockValid = (!nonClock && delayCounter - lastClock < (sampleRate * 2));	// Valid clock interval is within a second

	++delayCounter;
}

void Delay::GetSamples(Samples& samples)
{
	CheckForClock();					// Auto-detect if valid clock signal is present on delay CV

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
	delayTimePot = (0.95f * delayTimePot) + (0.05f * adc.delayTimePot);
	delayTimeCV = (0.95f * delayTimeCV) + (0.05f * adc.delayTimeCV);
	if (clockValid) {
		if (abs(hysteresisPotVal - adc.delayTimePot) > tempoHysteresis) {
			hysteresisPotVal = adc.delayTimePot;				// Store value for hysteresis checking
			delayMult = tempoMult[((float)adc.delayTimePot / 4096.0f) * tempoMult.size()];		// get tempo multiplier from lookup
			calcDelay = delayMult * (clockInterval / 2);
			while (calcDelay > (int32_t)audioBuffSize) {
				calcDelay /= 2;
			}
		}
	} else {
		const float combinedDelay = (4096.0f + delayTimePot - delayTimeCV) * 5.0f;		// Pot ADC increments, CV ADC decrements
		calcDelay = std::min((uint32_t)combinedDelay, audioBuffSize);
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

	// LED
	if (++ledCounter > (uint32_t)calcDelay) {
		ledCounter = 0;
		ledBrightness = 4095;
	}
	if (ledBrightness > 0) {
		clockedDelayLED = 0;
		unclockedDelayLED = 0;
		if (clockValid) {
			clockedDelayLED = ledBrightness;
		} else {
			unclockedDelayLED = ledBrightness;
		}
		--ledBrightness;
	}
}



void Delay::IdleJobs()
{
	lpFilter.Update(false);
}
