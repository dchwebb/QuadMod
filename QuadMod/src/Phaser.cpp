#include "Phaser.h"
#include <numbers>
#include <cmath>

Phaser phaser;

std::pair<float, float> Phaser::GetSamples(float* recordedSamples)
{
	float ph, channel0EndPhase = lfoPhase;

	// Go through each channel of audio that's passed in, applying one or more allpass filters
	// to each. Each channel will be treated identically in a (non-stereo) phaser, but we have
	// to have separate filter objects for each channel since the filters store the last few samples
	// passed through them.

	// Filters are stored with all channel 0 filters first, then all channel 1 filters, etc.

	for (int channel = 0; channel < 4; ++channel) {
		ph = lfoPhase;

		// For stereo phasing, keep the channels 90 degrees out of phase with each other
		if (stereo != 0 && channel != 0) {
			ph = fmodf(ph + 0.25, 1.0);
		}

		float out = recordedSamples[channel];

		// If feedback is enabled, include the feedback from the last sample in the
		// input of the allpass filter chain. This is actually not accurate to how
		// analog phasers work because there is a sample of delay between output and
		// input, which adds a further phase shift of up to 180 degrees at half the
		// sampling frequency. To truly model an analog phaser with feedback involves
		// modelling a delay-free loop, which is beyond the scope of this example.

		if (feedback != 0.0 && channel < numLastFilterOutputs) {
			out += feedback * lastFilterOutputs[channel];
		}

		// Filter the sample in place putting the output sample in place of the input
		for (int j = 0; j < filtersPerChannel; ++j) {
			// First, update the current allpass filter coefficients depending on the parameter settings and the LFO phase

			// Recalculating the filter coefficients is much more expensive than calculating
			// a sample. Only update the coefficients at a fraction of the sample rate; since
			// the LFO moves slowly, the difference won't generally be audible.
			if (false) {
				allpassFilters[channel * filtersPerChannel + j]->MakeAllpass(inverseSampleRate,	baseFrequency + sweepWidth * lfo(ph));
			}
			out = allpassFilters[channel * filtersPerChannel + j]->ProcessSample(out);
		}

		if (channel < numLastFilterOutputs) {
			lastFilterOutputs[channel] = out;
		}

		// Add the allpass signal to the output, though maintaining constant level
		// depth = 0 --> input only ; depth = 1 --> evenly balanced input and output
		out = (1.0f - 0.5f * depth) * recordedSamples[channel] + 0.5f * depth * out;

		// Update the LFO phase, keeping it in the range 0-1
		ph += lfoFrequency * inverseSampleRate;
		if (ph >= 1.0) {
			ph -= 1.0;
		}


		// Use channel 0 only to keep the phase in sync between calls to processBlock()
		// Otherwise quadrature phase on multiple channels will create problems.
		if (channel == 0) {
			channel0EndPhase = ph;
		}
	}

	lfoPhase = channel0EndPhase;

	float leftOut  = 0;
	float rightOut = 0;

	return std::make_pair(leftOut, rightOut);
}



// Function for calculating LFO waveforms. Phase runs from 0-1, output is scaled from 0 to 1
float Phaser::lfo(const float phase)
{
	if (phase < 0.25f) {
		return 0.5f + 2.0f * phase;
	} else if (phase < 0.75f) {
		return 1.0f - 2.0f * (phase - 0.25f);
	} else {
		return 2.0f * (phase - 0.75f);
	}
}

float AllpassFilter::ProcessSample(const float sample)
{
    // Process one sample, storing the last input and output
    y1 = (b0 * sample) + (b1 * x1) + (a1 * y1);
    x1 = sample;
    return y1;
}

void AllpassFilter::MakeAllpass(const float inverseSampleRate, const float centreFrequency)
{
    // This code based on calculations by Julius O. Smith:
    // https://ccrma.stanford.edu/~jos/pasp/Classic_Virtual_Analog_Phase.html

    // Avoid passing pi/2 to the tan function...
    const float w0 = std::min(centreFrequency * inverseSampleRate, 0.99f * (float)std::numbers::pi);
    const float tan_half_w0 = std::tan(0.5f * w0);

    b0 = a1 = (float)((1.0 - tan_half_w0) / (1.0 + tan_half_w0));
    b1 = -1.0f;
}
