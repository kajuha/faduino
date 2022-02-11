#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <string.h>

#include <iostream>
#include <queue>

#include "../../arduino/daemon/protocol_serial.h"
#include "serial.h"

ValueInput valueInput;

class Faduino {
private:
	std::string serial_port;
	int baudrate;
	std::queue<unsigned char> que;

	int fd;
	unsigned char buffer_rx[BUFSIZ];
public:
	Faduino(std::string serial_port, int baudrate) {		
		this->serial_port = serial_port;
		this->baudrate = baudrate;
	}

	bool initialize()
	{
		const char* COMM_PORT = serial_port.c_str();

		if(-1 == (fd = open(COMM_PORT, O_RDWR)))
		{
			printf("Error opening port\n");
			printf("Set port parameters using the following Linux command:\n stty -F %s %d raw\n", COMM_PORT, baudrate);
			printf("You may need to have ROOT access");
			return false;
		}

		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		
		switch(baudrate)
		{
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
				printf("Unsupported baudrate!");
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

	void closeSensor()
	{
		close(fd);
		printf("Closing faduino\n");
	}

	bool receiveData()
	{
		int rx_size;

		memset(buffer_rx, '\0', sizeof(buffer_rx));

		rx_size = read(fd, buffer_rx, BUFSIZ);

		for (int i=0; i<rx_size; i++) {
			que.push(buffer_rx[i]);
		}

		return true;
	}

	bool parseData() {
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
						packet[IDX_CRC16+i] = que.front();
						que.pop();
					}

					state = FSM_SERIAL::TAIL;
				}
				break;
			case FSM_SERIAL::TAIL:
				if (que.size() >= SIZE_TAIL) {
                    packet[IDX_TAIL] = que.front();
					if (packet[IDX_TAIL] == DATA_TAIL) {
						state = FSM_SERIAL::OK;
					} else {
						printf("FSM_SERIAL::TAIL not Match\n");
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

	void checksumData(unsigned char* packet)
	{
        // 수신부 crc16 문자열 추출
        unsigned short crc16out;
        sscanf((const char*)(packet+IDX_CRC16), "%04x", (unsigned int*)&crc16out);

        // 수신부 data의 crc16 계산
        unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);

        if (crc16out == crc16) {
            sscanf((const char*)(packet+IDX_DATA), "%01d%01d%01d%01d",
                &valueInput.estop_l, &valueInput.estop_r, &valueInput.sw_green, &valueInput.sw_red);
            #if 1
            packet[IDX_TYPE];
            *((int*)(packet+IDX_TS));
            valueInput.estop_l;
            valueInput.estop_r;
            valueInput.sw_green;
            valueInput.sw_red;
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

	int getQueueSize() {
		return que.size();
	}
};

int main(int argc, char* argv[]) {
	std::string serial_port = SERIAL_PORT;
	int baudrate = BAUDRATE;

	Faduino faduino(serial_port, baudrate);

	if(faduino.initialize() == false) {
		printf("Initialize() returns false, please check your devices.\n");
		printf("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB? %d raw\n", baudrate);
		printf("You may need to have ROOT access\n");
		return 0;
	} else {
		printf("faduino Initialization OK!\n");
	}

	while (true) {
		faduino.receiveData();
		faduino.parseData();
	}

	faduino.closeSensor();

	return 0;
}