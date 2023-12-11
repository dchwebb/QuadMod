#pragma once
#include "initialisation.h"
#include <tuple>


class AllpassFilter {
public:
	float ProcessSample(const float sampleToProcess);
	void MakeAllpass(const float inverseSampleRate, const float centreFrequency);
private:
	float x1, y1, b0, b1, a1;
};



class Phaser {
public:
	std::pair<float, float> GetSamples(float* recordedSamples);

private:
    float lfo(const float phase);

	// Adjustable parameters:
	float baseFrequency = 200.0;		// Lowest frequency of allpass filters
	float sweepWidth  = 2000.0;			// Amount of change from min to max delay
	float depth= 1.0;					// Mix level for phase-shifted signal (0-1)
	float feedback = 0.0;				// Feedback level for feedback phaser (0-<1)
	float lfoFrequency = 0.5;			// LFO frequency (Hz)
	int   filtersPerChannel = 4;		// How many allpass filters to use
	//int   waveform = kWaveformSine;		// What shape should be used for the LFO
	int   stereo = false;				// Whether to use stereo phasing

	float lfoPhase = 0.0;				// Phase of the low-frequency oscillator
	float inverseSampleRate  = 1.0 / 48000.0;; // It's more efficient to multiply than divide, so

	// Bank of allpass filters that do the phasing; N filters x M channels
	AllpassFilter **allpassFilters;

	// Storage of the last output sample from each bank of filters, for use in feedback loop
	float *lastFilterOutputs;
	int numLastFilterOutputs;
	int totalNumFilters;
};

extern Phaser phaser;




