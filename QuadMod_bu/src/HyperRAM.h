#pragma once

#include "initialisation.h"

class HyperRAM {
public:
	enum Register : uint8_t {IdRegister1 = 0x00, IdRegister2 = 0x01};
	enum Mode : uint8_t {Register = 0b101, Memory = 0b100};

	uint32_t Read(uint32_t address, Mode mode);
	uint32_t Write(uint32_t address, uint32_t val);

private:
	void MemMappedOff();
	bool memMapMode = false;
};

extern HyperRAM hyperRAM;
