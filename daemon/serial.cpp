#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <string.h>

#include <iostream>
#include <queue>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include "serial.h"

ValueInput valueInput;

Faduino::Faduino(std::string serialPort, int baudrate) {		
	this->serialPort = serialPort;
	this->baudrate = baudrate;
}

bool Faduino::initSerial()	{
	const char* COMM_PORT = serialPort.c_str();

	if(-1 == (fd = open(COMM_PORT, O_RDWR))) {
		printf("error opening port\n");
		printf("set port parameters using the following Linux command:\n stty -F %s %d raw\n", COMM_PORT, baudrate);
		printf("you may need to have ROOT access");
		return false;
	}

	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	
	switch(baudrate) {
		case 921600:
			newtio.c_cflag = B921600;
			break;
		case 576000:
			newtio.c_cflag = B576000;
			break;
		case 500000:
			newtio.c_cflag = B500000;
			break;
		case 460800:
			newtio.c_cflag = B460800;
			break;
		case 230400:
			newtio.c_cflag = B230400;
			break;
		case 115200:
			newtio.c_cflag = B115200;
			break;
		case 57600:
			newtio.c_cflag = B57600;
			break;
		case 38400:
			newtio.c_cflag = B38400;
			break;
		case 19200:
			newtio.c_cflag = B19200;
			break;
		case 9600:
			newtio.c_cflag = B9600;
			break;
		case 4800:
			newtio.c_cflag = B4800;
			break;
		default:
			printf("unsupported baudrate!");
			exit(0);
	}
	newtio.c_cflag |= CS8;
	newtio.c_cflag |= CLOCAL;
	newtio.c_cflag |= CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	#if 0
	newtio.c_cc[VMIN] = 1; 
	#else
	newtio.c_cc[VMIN] = 0;
	#endif

	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	printf("faduino communication port is ready\n");

	return true;
}

void Faduino::closeSerial() {
	close(fd);
	printf("closing faduino\n");
}

bool Faduino::sendFaduinoCmd(ValueOutput valueOutput) {
	static struct timeval time_now{};
	static uint32_t micros;
	gettimeofday(&time_now, nullptr);
	micros = (uint32_t)time_now.tv_usec;

	// 송신 포맷 생성
	serialBufferTx[IDX_HEAD] = DATA_HEAD;
	serialBufferTx[IDX_TYPE] = TYPE_CMD::CMD;
	*((uint32_t*)(serialBufferTx+IDX_TS)) = micros;
	memcpy(serialBufferTx+IDX_DATA, (char*)&valueOutput, SIZE_DATA_OUTPUT);
	// crc16 계산
	unsigned short crc16in = CRC::CRC16((unsigned char*)(serialBufferTx+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);
	sprintf((char*)(serialBufferTx+IDX_CRC16_OUTPUT), "%04x", crc16in);
	serialBufferTx[IDX_TAIL_OUTPUT] = DATA_TAIL;

	write(fd, serialBufferTx, SIZE_TOTAL_OUTPUT);

	return true;
}

bool Faduino::receiveFaduinoState(bool enableParsing) {
	int rx_size;

	memset(serialBufferRx, '\0', sizeof(serialBufferRx));

	rx_size = read(fd, serialBufferRx, BUFSIZ);

	for (int i=0; i<rx_size; i++) {
		queSerialRx.push(serialBufferRx[i]);
	}

	if (enableParsing) {
		parseFaduinoState();
	} else {
		if (queSerialRx.size()) {
			for (int i=0; i<rx_size; i++) {
				printf("[%02x]", queSerialRx.front());
				queSerialRx.pop();
			}
			printf("\n");
		}
	}

	return true;
}

bool Faduino::parseFaduinoState() {
	static int state = FSM_SERIAL::HEAD;
	static unsigned char packet[SIZE_TOTAL_INPUT] = {'\0', };

	switch (state) {
		case FSM_SERIAL::HEAD:
			if (queSerialRx.size() >= SIZE_HEAD) {
				packet[IDX_HEAD] = queSerialRx.front();
				if (packet[IDX_HEAD] == DATA_HEAD) {
					state = FSM_SERIAL::TYPE;
				} else {
					printf("FSM_SERIAL::HEAD not Match \n");
					state = FSM_SERIAL::HEAD;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_SERIAL::TYPE:
			if (queSerialRx.size() >= SIZE_TYPE) {
				packet[IDX_TYPE] = queSerialRx.front();
				state = FSM_SERIAL::TS;
				queSerialRx.pop();
			}
			break;
		case FSM_SERIAL::TS:
			if (queSerialRx.size() >= SIZE_TS) {
				for (int i=0; i<SIZE_TS; i++) {
					packet[IDX_TS+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_SERIAL::DATA;
			}
			break;
		case FSM_SERIAL::DATA:
			if (queSerialRx.size() >= SIZE_DATA_INPUT) {
				for (int i=0; i<SIZE_DATA_INPUT; i++) {
					packet[IDX_DATA+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_SERIAL::CRC16;
			}
			break;
		case FSM_SERIAL::CRC16:
			if (queSerialRx.size() >= SIZE_CRC16) {
				for (int i=0; i<SIZE_CRC16; i++) {
					packet[IDX_CRC16_INPUT+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_SERIAL::TAIL;
			}
			break;
		case FSM_SERIAL::TAIL:
			if (queSerialRx.size() >= SIZE_TAIL) {
				packet[IDX_TAIL_INPUT] = queSerialRx.front();
				if (packet[IDX_TAIL_INPUT] == DATA_TAIL) {
					state = FSM_SERIAL::OK;
				} else {
					printf("FSM_SERIAL::TAIL not Match\n");
					for (int i=0; i<SIZE_TOTAL_INPUT; i++) {
						printf("[%02x]", packet[i]);
					}
					printf("\n");
					state = FSM_SERIAL::HEAD;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_SERIAL::OK:
			checksumFaduinoState(packet);

			memset(packet, '\0', SIZE_TOTAL_INPUT);
			
			state = FSM_SERIAL::HEAD;

			break;
		default:
			state = FSM_SERIAL::HEAD;

			break;
	}
	
	return false;
}

void Faduino::checksumFaduinoState(unsigned char* packet) {
	// 수신부 crc16 문자열 추출
	unsigned short crc16out;
	sscanf((const char*)(packet+IDX_CRC16_INPUT), "%04x", (unsigned int*)&crc16out);

	// 수신부 data의 crc16 계산
	unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);

	if (crc16out == crc16) {
		switch (packet[IDX_TYPE]) {
			case TYPE_CMD::CMD:
				sscanf((const char*)(packet+IDX_DATA), "%01hd%01hd%01hd%01hd%01hd",
					&valueInput.estop_l, &valueInput.estop_r, &valueInput.sw_green, &valueInput.sw_red, &valueInput.sw_stop);

				queFaduinoState.push(valueInput);

				#if 0
				printf("type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				switch (valueInput.estop_l) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
					default:
						break;
				}
				switch (valueInput.estop_r) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
					default:
						break;
				}
				switch (valueInput.sw_green) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
						break;
					default:
						break;
				}
				switch (valueInput.sw_red) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
						break;
					default:
						break;
				}
				#endif

				#if 1
				printf("type:%d, ts:%d, ",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				printf("estop_l:%d, estop_r:%d, sw_green:%d, sw_red:%d, sw_stop:%d\n",
					valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red, valueInput.sw_stop);
				#endif
				break;
			case TYPE_CMD::SENSOR:
				static int sensorValue;
				sscanf((const char*)(packet+IDX_DATA), "%05d", &sensorValue);

				#if 1
				printf("type:%d, ts:%d, ", packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				printf("sensorValue:%d\n", sensorValue);
				#endif
				break;
			default:
				printf("unknown type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				break;
		}
	} else {
		printf("crc16 not matched !!!\n");
	}
}