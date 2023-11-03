#pragma once

#include "initialisation.h"

class AudioCodec {
public:
	void Init();
	uint8_t ReadData(uint16_t address);

private:
	struct __attribute__((__packed__)) Command {
		enum CommandType : uint8_t {Read  = 0x43, Write = 0xC3, activateSPI = 0xDE};
		enum RegisterAddress : uint16_t {PowerManagement = 0x00, AudioInterfaceFormat = 0x01, AnalogInput = 0x0B, SpiMode = 0xADDA};
		Command(CommandType rw, uint16_t address, uint8_t data) : data(data), address(address), cmdCode(rw) {}

		uint8_t data;
		uint16_t address;
		CommandType cmdCode;
	};

	void WriteData(uint16_t address, uint8_t data);
	void SendCmd(Command cmd);
};

extern AudioCodec audioCodec;
