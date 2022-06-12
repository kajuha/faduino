#include <stdio.h>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "serial.h"
#include "exec.h"

#define ARG_SERIAL_PORT 1
#define ARG_HOST_PORT 2
#define ARG_MAX (ARG_HOST_PORT+1)

Faduino faduino;

int sock, clientSock;

// 시그널 핸들러(클라이언트 TCP 해제)
#include <signal.h>
int readWriteInfinite = 1;
int clientOpen = 1;

void sigpipe_handler(int sig) {
	// signal(SIGPIPE, sigpipe_handler);
	printf("[s] received SIGPIPE: %d \n", sig);
	readWriteInfinite = 0;
}

void sigint_handler(int sig) {
	// signal(SIGINT, sigint_handler);
	printf("[s] received SIGINT: %d \n", sig);
	close(sock);
	clientOpen = 0;
}

std::queue<unsigned char> queTcpRx;

void checksumTcpState(unsigned char* packet) {
	#if 0
	// 수신부 crc16 문자열 추출
	unsigned short crc16out;
	sscanf((const char*)(packet+IDX_CRC16_OUTPUT), "%04x", (unsigned int*)&crc16out);

	// 수신부 data의 crc16 계산
	unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);

	if (crc16out == crc16) {
	#else
	// 수신부 crc16 문자열 추출
	uint32_t crc16out;
	memcpy(&crc16out, (const char*)(packet+IDX_CRC16_OUTPUT), SIZE_CRC16);
	printf("crc16out: %x\n", crc16out);

	if (crc16out == 0x55AA55AA) {
	#endif
		switch (packet[IDX_TYPE]) {
			case TYPE_CMD::RELAY:
				ValueOutput valueOutput;
        		memcpy(&valueOutput, packet+IDX_DATA, SIZE_DATA_OUTPUT);

				#if 1
				printf("[s] TS(us): %d\n", *((uint32_t*)(packet+IDX_TS)));
				printf("[s] led_green(on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.led_green.onTime, valueOutput.led_green.offTime,
					valueOutput.led_green.targetCount, valueOutput.led_green.lastState,
					valueOutput.led_green.update);
				printf("[s] led_red  (on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.led_red.onTime, valueOutput.led_red.offTime,
					valueOutput.led_red.targetCount, valueOutput.led_red.lastState,
					valueOutput.led_red.update);
				printf("[s] buzzer   (on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.buzzer.onTime, valueOutput.buzzer.offTime,
					valueOutput.buzzer.targetCount, valueOutput.buzzer.lastState,
					valueOutput.buzzer.update);
				printf("[s] led_start(on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.led_start.onTime, valueOutput.led_start.offTime,
					valueOutput.led_start.targetCount, valueOutput.led_start.lastState,
					valueOutput.led_start.update);
				printf("[s] led_stop (on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.led_stop.onTime, valueOutput.led_stop.offTime,
					valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState,
					valueOutput.led_stop.update);
				printf("[s] rel_break(on: %5d off: %5d count: %2d last: %1d update: %1d)\n",
					valueOutput.rel_break.onTime, valueOutput.rel_break.offTime,
					valueOutput.rel_break.targetCount, valueOutput.rel_break.lastState,
					valueOutput.rel_break.update);
				#endif

				faduino.sendFaduinoCmd(valueOutput);
				break;
			default:
				printf("[s] unknown type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				break;
		}
	} else {
		printf("[s] crc16 not matched !!!\n");
	}
}

bool parseTcpState() {
	static int state = FSM_FADUINO::HEAD;
	static unsigned char packet[SIZE_TOTAL_OUTPUT] = {'\0', };

	switch (state) {
		case FSM_FADUINO::HEAD:
			if (queTcpRx.size() >= SIZE_HEAD) {
				packet[IDX_HEAD] = queTcpRx.front();
				if (packet[IDX_HEAD] == DATA_HEAD) {
					state = FSM_FADUINO::TYPE;
				} else {
					printf("[s] FSM_FADUINO::HEAD not Match \n");
					state = FSM_FADUINO::HEAD;
				}
				queTcpRx.pop();
			}
			break;
		case FSM_FADUINO::TYPE:
			if (queTcpRx.size() >= SIZE_TYPE) {
				packet[IDX_TYPE] = queTcpRx.front();
				state = FSM_FADUINO::TS;
				queTcpRx.pop();
			}
			break;
		case FSM_FADUINO::TS:
			if (queTcpRx.size() >= SIZE_TS) {
				for (int i=0; i<SIZE_TS; i++) {
					packet[IDX_TS+i] = queTcpRx.front();
					queTcpRx.pop();
				}

				state = FSM_FADUINO::DATA;
			}
			break;
		case FSM_FADUINO::DATA:
			if (queTcpRx.size() >= SIZE_DATA_OUTPUT) {
				for (int i=0; i<SIZE_DATA_OUTPUT; i++) {
					packet[IDX_DATA+i] = queTcpRx.front();
					queTcpRx.pop();
				}

				state = FSM_FADUINO::CRC16;
			}
			break;
		case FSM_FADUINO::CRC16:
			if (queTcpRx.size() >= SIZE_CRC16) {
				for (int i=0; i<SIZE_CRC16; i++) {
					packet[IDX_CRC16_OUTPUT+i] = queTcpRx.front();
					queTcpRx.pop();
				}

				state = FSM_FADUINO::TAIL;
			}
			break;
		case FSM_FADUINO::TAIL:
			if (queTcpRx.size() >= SIZE_TAIL) {
				packet[IDX_TAIL_OUTPUT] = queTcpRx.front();
				if (packet[IDX_TAIL_OUTPUT] == DATA_TAIL) {
					state = FSM_FADUINO::OK;
				} else {
					printf("[s] FSM_FADUINO::TAIL not Match\n");
					for (int i=0; i<SIZE_TOTAL_OUTPUT; i++) {
						printf("[%02x]", packet[i]);
					}
					printf("\n");
					state = FSM_FADUINO::HEAD;
				}
				queTcpRx.pop();
			}
			break;
		case FSM_FADUINO::OK:
			checksumTcpState(packet);

			memset(packet, '\0', SIZE_TOTAL_OUTPUT);
			
			state = FSM_FADUINO::HEAD;

			break;
		default:
			state = FSM_FADUINO::HEAD;

			break;
	}
	
	return false;
}

void fThread(int* hostPort, bool* isSerial) {
	while (!*isSerial);

    // TCP 통신 관련 변수
    struct sockaddr_in addr, clientAddr;
    int addrLen;
    int ret;
    int recvLen;

    printf("[s] clientOpen while start (%d line)\n", __LINE__);
    while (clientOpen) {
        printf("[s] clientOpen while started (%d line)\n", __LINE__);

        // 소켓 열기
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            // perror("[s] socket open error, ");
            printf("[s] socket open error\n");
            // return;
            continue;
        }
        printf("[s] socket open\n");

        // 소켓 설정(TCP, 포트)
        memset(&addr, 0x00, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        printf("[s] hostPort: %d \n", *hostPort);
        addr.sin_port = htons(*hostPort);

        // time_wait 제거하기
        int option = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        // 소켓 설정 등록
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            // perror("[s] bind error, ");
            printf("[s] bind error\n");
            // return;
            usleep(500000);
            continue;
        }
        printf("[s] bind registerd\n");

        // 시그널 핸들러 등록
        // signal(SIGINT, sigint_handler);

        // 시그널 핸들러 등록
        readWriteInfinite = 1;
        signal(SIGPIPE, sigpipe_handler);

        // 리슨을 타임아웃으로 설정하고 싶을 경우
        struct timeval timeout;
#define TCP_TIMEOUT_SEC 3
        timeout.tv_sec = TCP_TIMEOUT_SEC;
        timeout.tv_usec = 0;

        if (setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
            // perror("[s] setsockopt failed, ");
            printf("[s] setsockopt failed\n");
        }

        // 연결 요청 대기
        if (listen(sock, 5) < 0) {
            // perror("[s] listen error, ");
            printf("[s] listen error\n");
            // return;
            usleep(500000);
            continue;
        } else {
            printf("[s] listen\n");
        }

        addrLen = sizeof(clientAddr);

        printf("[s] waiting for client..\n");

        printf("[s] socket : %d\n", sock);
        // 연결 수락
        clientSock = accept(sock, (struct sockaddr *)&clientAddr, (socklen_t*)&addrLen);
        if (clientSock < 0) {
            // perror("[s] accept error, ");
            printf("[s] accept error\n");
            // goto PROGRAM_END;
            // return;
            // printf("[s] socket : %d\n", sock);
            // printf("[s] clientSock: %d\n", clientSock);
            close(clientSock);
            close(sock);
            usleep(500000);
            continue;
        } else {
            printf("[s] clinet ip : %s\n", inet_ntoa(clientAddr.sin_addr));
            printf("[s] client accept\n");
        }

        struct timeval time_now{};
        gettimeofday(&time_now, nullptr);
        time_t ts_now;
        ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

        uint8_t recvBuffer[BUFSIZ] = {'\0', };

        printf("[s] readWriteInfinite while start (%d line)\n", __LINE__);
        while (readWriteInfinite) {
            // printf("[s] readWriteInfinite while started (%d line)\n", __LINE__);

			recvLen = recv(clientSock, recvBuffer, BUFSIZ, MSG_PEEK|MSG_DONTWAIT);
			if (recvLen > 0) {
				// 버퍼개수 읽기(MSG_PEEK는 버퍼확인용)
				memset(recvBuffer, '\0', sizeof(recvBuffer));
				recvLen = recv(clientSock, recvBuffer, BUFSIZ, 0);
				for (int i=0; i<recvLen; i++) {
					queTcpRx.push(recvBuffer[i]);
				}
				#if 1
				printf("recvLen: %d ", recvLen);
				for (int i=0; i<recvLen; i++) {
					printf("[%02x]", recvBuffer[i]);
				}
				printf("\n");
				#endif
			} else if (recvLen == 0) {
				// 참고자료
				// recv를 호출했을때 0이 리턴되거나 send를 호출했을때 -1로 에러리턴이면 상대편 연결이 종료된 것

				// 커넥션중일 때는 MSG_PEEK|MSG_DONTWAIT에서 아무것도 송신안할 경우 -1이 발생하였음
				// 커넥션을 강제해제했을 경우 0이 발생하였음
				// MSG_DONTWIT를 추가하지 않았을 경우에는 무한대기가 되었음
				// 우선 이케이스에는 0을 커넥션 종료로 처리함
				printf("[s] client disconnect error, recv: %d \n", recvLen);
				readWriteInfinite = 0;
				continue;
			} else {
			}

			parseTcpState();
        }

        printf("[s] tcp read/write end\n");

        ret = close(clientSock);
        printf("[s] client socket closed, ret: %d\n", ret);

        ret = close(sock);
        printf("[s] socket closed, ret: %d\n", ret);
        // 소켓을 정상적으로 너무 빨리 닫고 재 열기할 경우
        // 해당 포트가 이미 사용중이라는 표시가 나타날 수 있음
        usleep(500000);
    }

    printf("[s] program end\n");
}

int main(int argc, char* argv[]) {
    int hostPort;
	bool isSerial = false;

    if (argc != ARG_MAX) {
        printf("!!! requirement arguments is %d ea !!!\n", ARG_MAX);
        printf("input argument: %d\n", argc);
        for (int i=0; i<argc; i++) {
            printf("argv[%d]: %s\n", i, argv[i]);
        }

        return -1;
    }
    // argument(host port)
    char* p;
    errno = 0;
    hostPort = strtol(argv[ARG_HOST_PORT], &p, 10);
    if (*p != '\0' || errno != 0) {
        printf("argument(host port) parsing error.\n");

        return -1;
    }
    printf("argument(host port) is set :[%d].\n", hostPort);

    boost::thread threadTcp(fThread, &hostPort, &isSerial);

	std::string serialPort = argv[ARG_SERIAL_PORT];
	int baudrate = BAUDRATE;

	ValueInput valueInput, valueInputPre;
	ValueOutput valueOutput, valueOutputPre;

	printf("serial port: %s\n", serialPort.c_str());

	faduino = Faduino(serialPort, baudrate);
	Prog prog;

	if(faduino.initSerial() == false) {
		printf("init() returns false, please check your devices.\n");
		printf("Set port parameters using the following Linux command:\n");
		printf("stty -F /dev/ttyUSB? %d raw\n", baudrate);
		printf("You may need to have ROOT access\n");

		return 0;
	}

	// faduinod와 faduino간 연결 확인후 다음 진행할 것
	// faduino의 ack 명령을 추가할 것
	// sleep는 아래 명령이 빠르게 패스되어 임시로 사용
	sleep(1);

	// PC가 정상적으로 켜졌을 경우 비프 및 LED 상태 알림
	// faduino 부팅시 GREEN/RED LED ON/OFF(1000,500) 무한 반복 및 비프음(500,200) 5회
	// daemon 시작시 GREEN/RED LED ON/OFF(500,1000) 5회 및 비프음(200,500) 5회
	valueOutput.led_green.onTime = 500;
	valueOutput.led_green.offTime = 1000;
	valueOutput.led_green.targetCount = STATE_ACT::T5;
	valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
	valueOutput.led_red.onTime = 500;
	valueOutput.led_red.offTime = 1000;
	valueOutput.led_red.targetCount = STATE_ACT::T5;
	valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
	valueOutput.buzzer.onTime = 200;
	valueOutput.buzzer.offTime = 500;
	valueOutput.buzzer.targetCount = STATE_ACT::T5;
	valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
	valueOutput.led_start.onTime = 1000;
	valueOutput.led_start.offTime = 500;
	valueOutput.led_start.targetCount = STATE_ACT::T10;
	valueOutput.led_start.lastState = FADUINO::RELAY::ON;
	valueOutput.led_stop.onTime = 500;
	valueOutput.led_stop.offTime = 1000;
	valueOutput.led_stop.targetCount = STATE_ACT::T10;
	valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
	valueOutput.rel_break.onTime = 0;
	valueOutput.rel_break.offTime = 0;
	valueOutput.rel_break.targetCount = STATE_ACT::INFINITE;
	valueOutput.rel_break.lastState = FADUINO::RELAY::OFF;
	valueOutput.led_green.update = 1;
	valueOutput.led_red.update = 1;
	valueOutput.buzzer.update = 1;
	valueOutput.led_start.update = 1;
	valueOutput.led_stop.update = 1;
	valueOutput.rel_break.update = 0;
	faduino.sendFaduinoCmd(valueOutput);
    
    printf("rosOpenCmd: %s\n", ROS_RUN);
    printf("rosCheckCmd: %s\n", ROS_CHECK);
    printf("rosCloseCmd: %s\n", ROS_KILL);

	isSerial = true;

	while (true) {
		usleep(100);
		faduino.receiveFaduinoState();

		static int sizeFaduinoState;
		sizeFaduinoState = faduino.queFaduinoState.size();
		if (sizeFaduinoState) {
			printf("faduino.queFaduinoState.size(): %d\n", sizeFaduinoState);
			valueInput = faduino.queFaduinoState.front();
			faduino.queFaduinoState.pop();

			printf("valueInput   : %1d %1d %1d %1d %1d\nvalueInputPre: %1d %1d %1d %1d %1d\n",
				valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red, valueInput.sw_stop,
				valueInputPre.estop_l, valueInputPre.estop_r, valueInputPre.sw_green, valueInputPre.sw_red, valueInputPre.sw_stop);
			
			if (valueInputPre.estop_l != valueInput.estop_l ||
				valueInputPre.estop_r != valueInput.estop_r ||
				valueInputPre.sw_green != valueInput.sw_green ||
				valueInputPre.sw_red != valueInput.sw_red ||
				valueInputPre.sw_stop != valueInput.sw_stop) {
				valueInputPre = valueInput;

				switch (valueInput.estop_l) {
					case PUSHED:
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.led_green.onTime = 0;
						valueOutput.led_green.offTime = 0;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 1000;
						valueOutput.led_red.offTime = 0;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 1000;
						valueOutput.buzzer.offTime = 0;
						valueOutput.buzzer.targetCount = STATE_ACT::INFINITE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						printf("valueInput.estop_l PUSHED\n");
						valueOutputPre = valueOutput;
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
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.led_green.onTime = 0;
						valueOutput.led_green.offTime = 0;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 1000;
						valueOutput.led_red.offTime = 0;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 1000;
						valueOutput.buzzer.offTime = 0;
						valueOutput.buzzer.targetCount = STATE_ACT::INFINITE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						printf("valueInput.estop_r PUSHED\n");
						valueOutputPre = valueOutput;
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
						#if 1
						valueOutput.led_green.onTime = 1000;
						valueOutput.led_green.offTime = 0;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 0;
						valueOutput.led_red.offTime = 1000;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 500;
						valueOutput.buzzer.offTime = 500;
						valueOutput.buzzer.targetCount = STATE_ACT::ONCE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						printf("valueInput.sw_green DOUBLE\n");

						if (!strlen(prog.execute(ROS_CHECK).c_str())) {
							prog.rosRun();
							printf("run ROS\n");
						} else {
							valueOutput.led_green.onTime = 1000;
							valueOutput.led_green.offTime = 0;
							valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
							valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
							valueOutput.led_red.onTime = 0;
							valueOutput.led_red.offTime = 1000;
							valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
							valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
							valueOutput.buzzer.onTime = 300;
							valueOutput.buzzer.offTime = 200;
							valueOutput.buzzer.targetCount = STATE_ACT::TWICE;
							valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
							valueOutput.led_green.update = 1;
							valueOutput.led_red.update = 1;
							valueOutput.buzzer.update = 1;
							valueOutput.led_start.update = 0;
							valueOutput.led_stop.update = 0;
							valueOutput.rel_break.update = 0;
							faduino.sendFaduinoCmd(valueOutput);
							printf("already run ROS\n");
						}
						#endif
						break;
					case LONG:
						#if 1
						// 임시로 오동작일 경우 부저 및 LED 끄는 용도
						valueOutput.led_green.onTime = 0;
						valueOutput.led_green.offTime = 1000;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 0;
						valueOutput.led_red.offTime = 1000;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 0;
						valueOutput.buzzer.offTime = 1000;
						valueOutput.buzzer.targetCount = STATE_ACT::INFINITE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						printf("valueInput.sw_green LONG\n");
						valueOutputPre = valueOutput;
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
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.led_green.onTime = 0;
						valueOutput.led_green.offTime = 1000;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 1000;
						valueOutput.led_red.offTime = 0;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 500;
						valueOutput.buzzer.offTime = 500;
						valueOutput.buzzer.targetCount = STATE_ACT::TWICE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						printf("valueInput.sw_red DOUBLE\n");

						prog.execute(ROS_KILL);
						printf("kill ROS\n");
						#endif
						break;
					case LONG:
						#if 1
						// 임시로 오동작일 경우 부저 및 LED 끄는 용도
						valueOutput.led_green.onTime = 0;
						valueOutput.led_green.offTime = 1000;
						valueOutput.led_green.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_green.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_red.onTime = 0;
						valueOutput.led_red.offTime = 1000;
						valueOutput.led_red.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_red.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.onTime = 0;
						valueOutput.buzzer.offTime = 1000;
						valueOutput.buzzer.targetCount = STATE_ACT::INFINITE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 1;
						valueOutput.led_red.update = 1;
						valueOutput.buzzer.update = 1;
						valueOutput.led_start.update = 0;
						valueOutput.led_stop.update = 0;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						printf("valueInput.sw_red LONG\n");
						valueOutputPre = valueOutput;
						#endif
						break;
					default:
						break;
				}
				switch (valueInput.sw_stop) {
					case PUSHED:
						break;
					case RELEASED:
						break;
					case DOUBLE:
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.led_start.onTime = 1000;
						valueOutput.led_start.offTime = 500;
						valueOutput.led_start.targetCount = STATE_ACT::T10;
						valueOutput.led_start.lastState = FADUINO::RELAY::ON;
						valueOutput.led_stop.onTime = 500;
						valueOutput.led_stop.offTime = 1000;
						valueOutput.led_stop.targetCount = STATE_ACT::T10;
						valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
						valueOutput.rel_break.onTime = 0;
						valueOutput.rel_break.offTime = 0;
						valueOutput.rel_break.targetCount = STATE_ACT::INFINITE;
						valueOutput.rel_break.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_green.update = 0;
						valueOutput.led_red.update = 0;
						valueOutput.buzzer.update = 0;
						valueOutput.led_start.update = 1;
						valueOutput.led_stop.update = 1;
						valueOutput.rel_break.update = 0;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						printf("valueInput.sw_stop DOUBLE\n");
						#endif
						break;
					case LONG:
						break;
					default:
						break;
				}
			}
		}
	}

	faduino.closeSerial();

    printf("[c] threadTcp join\n");
    threadTcp.join();
    printf("[c] threadTcp joined\n");

	return 0;
}