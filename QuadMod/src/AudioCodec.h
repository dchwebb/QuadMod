#pragma once

#include "initialisation.h"

class AudioCodec {
public:
	void Init();


private:
	struct Command_t {
		enum CommandType : uint8_t {Read  = 0x43, Write = 0xC3};
		enum RegisterAddress : uint16_t {AnalogInput = 0x0B};
		Command_t(CommandType rw, uint16_t address, uint8_t data) : data(data), address(address), cmdCode(rw) {}

		uint8_t data;
		uint8_t address;
		CommandType cmdCode;
	};

	void SendCmd(Command_t cmd);
};

extern AudioCodec audioCodec;
