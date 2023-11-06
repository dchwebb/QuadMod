#pragma once

#include "initialisation.h"

class HyperRAM {
public:
	enum Register : uint8_t {IdRegister1 = 0x00, IdRegister2 = 0x01};

	uint32_t GetID();

private:
	void MemMappedOff();
	bool memMapMode = false;
};

extern HyperRAM hyperRAM;
