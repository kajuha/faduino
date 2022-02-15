#include <stdio.h>
#include <string>

#include "serial.h"

#define SERIAL_PORT "/dev/ttyUSB0"

int main(int argc, char* argv[]) {
	std::string serial_port = SERIAL_PORT;
	int baudrate = BAUDRATE;

	Faduino faduino(serial_port, baudrate);

	if(faduino.init() == false) {
		printf("init() returns false, please check your devices.\n");
		printf("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB? %d raw\n", baudrate);
		printf("You may need to have ROOT access\n");
		return 0;
	} else {
		printf("faduino Initialization OK!\n");
	}

	while (true) {
		#if 0
		faduino.receiveData();
		#else
		faduino.receiveData(true);
		#endif
	}

	faduino.closeDevice();

	return 0;
}