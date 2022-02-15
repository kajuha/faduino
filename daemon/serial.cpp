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

Faduino::Faduino(std::string serial_port, int baudrate) {		
	this->serial_port = serial_port;
	this->baudrate = baudrate;
}

bool Faduino::init()	{
	const char* COMM_PORT = serial_port.c_str();

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

void Faduino::closeDevice() {
	close(fd);
	printf("closing faduino\n");
}

bool Faduino::sendData(ValueOutput valueOutput) {
	static struct timeval time_now{};
	static uint32_t micros;
	gettimeofday(&time_now, nullptr);
	micros = (uint32_t)time_now.tv_usec;

	// 송신 포맷 생성
	buffer_tx[IDX_HEAD] = DATA_HEAD;
	buffer_tx[IDX_TYPE] = TYPE_CMD::CMD;
	*((uint32_t*)(buffer_tx+IDX_TS)) = micros;
	memcpy(buffer_tx+IDX_DATA, (char*)&valueOutput, SIZE_DATA_OUTPUT);
	// crc16 계산
	unsigned short crc16in = CRC::CRC16((unsigned char*)(buffer_tx+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);
	sprintf((char*)(buffer_tx+IDX_CRC16_OUTPUT), "%04x", crc16in);
	buffer_tx[IDX_TAIL_OUTPUT] = DATA_TAIL;

	write(fd, buffer_tx, SIZE_TOTAL_OUTPUT);

	return true;
}

bool Faduino::receiveData(bool enableParsing) {
	int rx_size;

	memset(buffer_rx, '\0', sizeof(buffer_rx));

	rx_size = read(fd, buffer_rx, BUFSIZ);

	for (int i=0; i<rx_size; i++) {
		que.push(buffer_rx[i]);
	}

	if (enableParsing) {
		parseData();
	} else {
		if (que.size()) {
			for (int i=0; i<rx_size; i++) {
				printf("[%02x]", que.front());
				que.pop();
			}
			printf("\n");
		}
	}

	return true;
}

bool Faduino::parseData() {
	static int state = FSM_SERIAL::HEAD;
	static unsigned char packet[SIZE_TOTAL_INPUT] = {'\0', };

	switch (state) {
		case FSM_SERIAL::HEAD:
			if (que.size() >= SIZE_HEAD) {
				packet[IDX_HEAD] = que.front();
				if (packet[IDX_HEAD] == DATA_HEAD) {
					state = FSM_SERIAL::TYPE;
				} else {
					printf("FSM_SERIAL::HEAD not Match \n");
					state = FSM_SERIAL::HEAD;
				}
				que.pop();
			}
			break;
		case FSM_SERIAL::TYPE:
			if (que.size() >= SIZE_TYPE) {
				packet[IDX_TYPE] = que.front();
				state = FSM_SERIAL::TS;
				que.pop();
			}
			break;
		case FSM_SERIAL::TS:
			if (que.size() >= SIZE_TS) {
				for (int i=0; i<SIZE_TS; i++) {
					packet[IDX_TS+i] = que.front();
					que.pop();
				}

				state = FSM_SERIAL::DATA;
			}
			break;
		case FSM_SERIAL::DATA:
			if (que.size() >= SIZE_DATA_INPUT) {
				for (int i=0; i<SIZE_DATA_INPUT; i++) {
					packet[IDX_DATA+i] = que.front();
					que.pop();
				}

				state = FSM_SERIAL::CRC16;
			}
			break;
		case FSM_SERIAL::CRC16:
			if (que.size() >= SIZE_CRC16) {
				for (int i=0; i<SIZE_CRC16; i++) {
					packet[IDX_CRC16_INPUT+i] = que.front();
					que.pop();
				}

				state = FSM_SERIAL::TAIL;
			}
			break;
		case FSM_SERIAL::TAIL:
			if (que.size() >= SIZE_TAIL) {
				packet[IDX_TAIL_INPUT] = que.front();
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
				que.pop();
			}
			break;
		case FSM_SERIAL::OK:
			checksumData(packet);

			memset(packet, '\0', SIZE_TOTAL_INPUT);
			
			state = FSM_SERIAL::HEAD;

			break;
		default:
			state = FSM_SERIAL::HEAD;

			break;
	}
	
	return false;
}

void Faduino::checksumData(unsigned char* packet) {
	// 수신부 crc16 문자열 추출
	unsigned short crc16out;
	sscanf((const char*)(packet+IDX_CRC16_INPUT), "%04x", (unsigned int*)&crc16out);

	// 수신부 data의 crc16 계산
	unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);

	if (crc16out == crc16) {
		sscanf((const char*)(packet+IDX_DATA), "%01hd%01hd%01hd%01hd",
			&valueInput.estop_l, &valueInput.estop_r,
			&valueInput.sw_green, &valueInput.sw_red);
		#if 1
		printf("type:%d, ts:%d\n",
			packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
		switch (valueInput.estop_l) {
			case PUSHED:
				#if 0
				ValueOutput valueOutput;
				valueOutput.led_green.on = 0;
				valueOutput.led_green.off = 1000;
				valueOutput.led_red.on = 0;
				valueOutput.led_red.off = 1000;
				valueOutput.buzzer.on = 500;
				valueOutput.buzzer.off = 500;
				sendData(valueOutput);
				#endif
				break;
			case RELEASED:
				break;
			case DOUBLE:
				break;
			case LONG:
				break;
			default:
				break;
		}
		switch (valueInput.estop_r) {
			case PUSHED:
				#if 0
				ValueOutput valueOutput;
				valueOutput.led_green.on = 0;
				valueOutput.led_green.off = 1000;
				valueOutput.led_red.on = 0;
				valueOutput.led_red.off = 1000;
				valueOutput.buzzer.on = 1000;
				valueOutput.buzzer.off = 1000;
				sendData(valueOutput);
				#endif
				break;
			case RELEASED:
				break;
			case DOUBLE:
				break;
			case LONG:
				break;
			default:
				break;
		}
		switch (valueInput.sw_green) {
			case PUSHED:
				break;
			case RELEASED:
				break;
			case DOUBLE:
				#if 0
				ValueOutput valueOutput;
				valueOutput.led_green.on = 500;
				valueOutput.led_green.off = 500;
				valueOutput.led_red.on = 500;
				valueOutput.led_red.off = 500;
				valueOutput.buzzer.on = 500;
				valueOutput.buzzer.off = 500;
				valueOutput.buzzer.act = 2;
				sendData(valueOutput);
				#endif
				break;
			case LONG:
				#if 0
				valueOutput.led_green.on = 0;
				valueOutput.led_green.off = 1000;
				valueOutput.led_red.on = 0;
				valueOutput.led_red.off = 1000;
				valueOutput.buzzer.on = 0;
				valueOutput.buzzer.off = 1000;
				valueOutput.buzzer.act = STATE_ACT::INFINITE;
				sendData(valueOutput);
				#endif
				break;
			default:
				break;
		}
		switch (valueInput.sw_red) {
			case PUSHED:
				break;
			case RELEASED:
				break;
			case DOUBLE:
				#if 0
				ValueOutput valueOutput;
				valueOutput.led_green.on = 1000;
				valueOutput.led_green.off = 1000;
				valueOutput.led_red.on = 1000;
				valueOutput.led_red.off = 1000;
				valueOutput.buzzer.on = 1000;
				valueOutput.buzzer.off = 1000;
				valueOutput.buzzer.act = 3;
				sendData(valueOutput);
				#endif
				break;
			case LONG:
				#if 0
				valueOutput.led_green.on = 0;
				valueOutput.led_green.off = 1000;
				valueOutput.led_red.on = 0;
				valueOutput.led_red.off = 1000;
				valueOutput.buzzer.on = 0;
				valueOutput.buzzer.off = 1000;
				valueOutput.buzzer.act = STATE_ACT::INFINITE;
				sendData(valueOutput);
				#endif
				break;
			default:
				break;
		}
		#else
		printf("type:%d, ts:%d\n",
			packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
		printf("estop_l:%d, estop_r:%d, sw_green:%d, sw_red:%d\n",
			valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red);
		#endif
	} else {
		printf("crc16 not matched !!!\n");
	}
}

int Faduino::getQueueSize() {
	return que.size();
}