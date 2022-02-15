#pragma once

#include <string>
#include <queue>

#include "../arduino/daemon/protocol_serial.h"

class Faduino {
private:
	std::string serial_port;
	int baudrate;
	std::queue<unsigned char> que;

	int fd;
	unsigned char buffer_rx[BUFSIZ];
	unsigned char buffer_tx[BUFSIZ];
public:
	Faduino(std::string serial_port, int baudrate);
	bool init();
	void closeDevice();
	bool sendData(ValueOutput valueOutput);
	bool receiveData(bool enableParsing=true);
	bool parseData();
	void checksumData(unsigned char* packet);
	int getQueueSize();
};