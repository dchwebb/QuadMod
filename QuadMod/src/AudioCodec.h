#pragma once

#include "initialisation.h"
#include "GpioPin.h"

class AudioCodec {
public:
	void Init();
	uint8_t ReadData(uint16_t address);
	void WriteData(uint16_t address, uint8_t data);
	void TestOutput();

	struct {
		uint32_t channel1;
		uint32_t channel2;
		uint32_t channel3;
		uint32_t channel4;
		bool leftRight = true;		// to keep count of which channel we are receiving
	} dataIn;

	static constexpr uint32_t audioBuffSize = 34000;
	uint16_t buffPos = 0;
	uint16_t playBuff = 10000;
	float audioBuffer[4][audioBuffSize] = {};

private:
	struct __attribute__((__packed__)) Command {
		enum CommandType : uint8_t {Read  = 0x43, Write = 0xC3, activateSPI = 0xDE};
		enum RegisterAddress : uint16_t {PowerManagement = 0x00, AudioInterfaceFormat = 0x01, AnalogInput = 0x0B, AnalogFilter = 0x0A, SpiMode = 0xADDA};
		Command(CommandType rw, uint16_t address, uint8_t data) : data(data), address(address), cmdCode(rw) {}

		uint8_t data;
		uint16_t address;
		CommandType cmdCode;
	};

	void SendCmd(Command cmd);

	GpioPin pdnPin{GPIOD, 15, GpioPin::Type::Output};		// PDN pin is used to bring Codec out of reset
};

extern AudioCodec audioCodec;
