#pragma once

#include <string>
#include <queue>

#include "../arduino/daemon/protocol_serial.h"

class Faduino {
public:
	std::string serialPort;
	int baudrate;
	std::queue<unsigned char> queSerialRx;

	std::queue<ValueOutput> queFaduinoCmd;
	std::queue<ValueInput> queFaduinoState;

	int fd;
	unsigned char serialBufferRx[BUFSIZ];
	unsigned char serialBufferTx[BUFSIZ];
	
	Faduino(std::string serialPort, int baudrate);

	bool initSerial();
	void closeSerial();
	bool sendFaduinoCmd(ValueOutput valueOutput);
	bool receiveFaduinoState(bool enableParsing=true);
	bool parseFaduinoState();
	void checksumFaduinoState(unsigned char* packet);
};