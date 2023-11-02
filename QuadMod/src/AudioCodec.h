#pragma once

#include "initialisation.h"

class AudioCodec {
public:
	void Init();


private:
	struct __attribute__((__packed__)) Command {
		enum CommandType : uint8_t {Read  = 0x43, Write = 0xC3};
		enum RegisterAddress : uint16_t {PowerManagement = 0x00, AudioInterfaceFormat = 0x01, AnalogInput = 0x0B};
		Command(CommandType rw, uint16_t address, uint8_t data) : data(data), address(address), cmdCode(rw) {}

		uint8_t data;
		uint16_t address;
		CommandType cmdCode;
	};

	void WriteData(uint16_t address, uint8_t data);
	uint8_t ReadData(uint16_t address);
};

extern AudioCodec audioCodec;
