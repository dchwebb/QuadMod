#include "Flanger.h"
#include "Cordic.h"
#include <cmath>

// Code loosely based on:
// https://code.soundsoftware.ac.uk/projects/audio_effects_textbook_code/repository/show/effects/flanger/Source

Flanger flanger;

void Flanger::GetSamples(Samples& samples)
{
	if (++writePos == audioBuffSize)   { writePos = 0; }
	if (++readPos == audioBuffSize)    { readPos = 0; }

	Samples newSample = audioBuffer[readPos];
	audioBuffer[writePos] = samples;

	for (uint32_t channel = 0; channel < 4; ++channel) {

	}
}



void Flanger::IdleJobs()
{

}


