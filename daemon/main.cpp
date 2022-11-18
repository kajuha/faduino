#include <stdio.h>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "reprintf.h"
#include "serial.h"
#include "exec.h"

#define ARG_SERIAL_PORT 1
#define ARG_TCP_PORT 2
#define ARG_MAX (ARG_TCP_PORT+1)

Faduino faduino;

int sock, clientSock;

// 시그널 핸들러(클라이언트 TCP 해제)
#include <signal.h>
int readWriteInfinite = 1;
int clientOpen = 1;

void sigpipe_handler(int sig) {
	// signal(SIGPIPE, sigpipe_handler);
	reprintf(ScreenOutput::ALWAYS, "[s] received SIGPIPE: %d \n", sig);
	readWriteInfinite = 0;
}

void sigint_handler(int sig) {
	// signal(SIGINT, sigint_handler);
	reprintf(ScreenOutput::ALWAYS, "[s] received SIGINT: %d \n", sig);
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
	reprintf(ScreenOutput::NO, "crc16out: %x\n", crc16out);

	if (crc16out == 0x55AA55AA) {
	#endif
		switch (packet[IDX_TYPE]) {
			case TYPE_CMD::RELAY:
				ValueOutput valueOutput;
        		memcpy(&valueOutput, packet+IDX_DATA, SIZE_DATA_OUTPUT);

				#if 1
				reprintf(ScreenOutput::NO, "[s] TS(us): %d\n", *((uint32_t*)(packet+IDX_TS)));
				reprintf(ScreenOutput::NO, "[s] buzzer   (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.buzzer.onTime, valueOutput.buzzer.offTime,
					valueOutput.buzzer.targetCount, valueOutput.buzzer.lastState,
					valueOutput.buzzer.update, valueOutput.buzzer.order);
				reprintf(ScreenOutput::NO, "[s] md_power (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.md_power.onTime, valueOutput.md_power.offTime,
					valueOutput.md_power.targetCount, valueOutput.md_power.lastState,
					valueOutput.md_power.update, valueOutput.md_power.order);
				reprintf(ScreenOutput::NO, "[s] md_estop (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.md_estop.onTime, valueOutput.md_estop.offTime,
					valueOutput.md_estop.targetCount, valueOutput.md_estop.lastState,
					valueOutput.md_estop.update, valueOutput.md_estop.order);
				reprintf(ScreenOutput::NO, "[s] led_start(on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.led_start.onTime, valueOutput.led_start.offTime,
					valueOutput.led_start.targetCount, valueOutput.led_start.lastState,
					valueOutput.led_start.update, valueOutput.led_start.order);
				reprintf(ScreenOutput::NO, "[s] led_stop (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.led_stop.onTime, valueOutput.led_stop.offTime,
					valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState,
					valueOutput.led_stop.update, valueOutput.led_stop.order);
				reprintf(ScreenOutput::NO, "[s] bat_relay(on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
					valueOutput.bat_relay.onTime, valueOutput.bat_relay.offTime,
					valueOutput.bat_relay.targetCount, valueOutput.bat_relay.lastState,
					valueOutput.bat_relay.update, valueOutput.bat_relay.order);
				#endif

				faduino.sendFaduinoCmd(valueOutput);
				break;
			default:
				reprintf(ScreenOutput::NO, "[s] unknown type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				break;
		}
	} else {
		reprintf(ScreenOutput::NO, "[s] crc16 not matched !!!\n");
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
					reprintf(ScreenOutput::NO, "[s] FSM_FADUINO::HEAD not Match \n");
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
					reprintf(ScreenOutput::NO, "[s] FSM_FADUINO::TAIL not Match\n");
					for (int i=0; i<SIZE_TOTAL_OUTPUT; i++) {
						reprintf(ScreenOutput::NO, "[%02x]", packet[i]);
					}
					reprintf(ScreenOutput::NO, "\n");
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

void fThread(int* tcpPort, bool* isSerial) {
	while (!*isSerial);

    // TCP 통신 관련 변수
    struct sockaddr_in addr, clientAddr;
    int addrLen;
    int ret;
    int recvLen;

    reprintf(ScreenOutput::NO, "[s] clientOpen while start (%d line)\n", __LINE__);
    while (clientOpen) {
        reprintf(ScreenOutput::NO, "[s] clientOpen while started (%d line)\n", __LINE__);

        // 소켓 열기
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            // perror("[s] socket open error, ");
            reprintf(ScreenOutput::NO, "[s] socket open error\n");
            // return;
            continue;
        }
        reprintf(ScreenOutput::NO, "[s] socket open\n");

        // 소켓 설정(TCP, 포트)
        memset(&addr, 0x00, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        reprintf(ScreenOutput::NO, "[s] tcpPort: %d \n", *tcpPort);
        addr.sin_port = htons(*tcpPort);

        // time_wait 제거하기
        int option = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        // 소켓 설정 등록
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            // perror("[s] bind error, ");
            reprintf(ScreenOutput::NO, "[s] bind error\n");
            // return;
            usleep(500000);
            continue;
        }
        reprintf(ScreenOutput::NO, "[s] bind registerd\n");

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
            reprintf(ScreenOutput::NO, "[s] setsockopt failed\n");
        }

        // 연결 요청 대기
        if (listen(sock, 5) < 0) {
            // perror("[s] listen error, ");
            reprintf(ScreenOutput::NO, "[s] listen error\n");
            // return;
            usleep(500000);
            continue;
        } else {
            reprintf(ScreenOutput::NO, "[s] listen\n");
        }

        addrLen = sizeof(clientAddr);

        reprintf(ScreenOutput::NO, "[s] waiting for client..\n");

        reprintf(ScreenOutput::NO, "[s] socket : %d\n", sock);
        // 연결 수락
        clientSock = accept(sock, (struct sockaddr *)&clientAddr, (socklen_t*)&addrLen);
        if (clientSock < 0) {
            // perror("[s] accept error, ");
            reprintf(ScreenOutput::NO, "[s] accept error\n");
            // goto PROGRAM_END;
            // return;
            // reprintf(ScreenOutput::NO, "[s] socket : %d\n", sock);
            // reprintf(ScreenOutput::NO, "[s] clientSock: %d\n", clientSock);
            close(clientSock);
            close(sock);
            usleep(500000);
            continue;
        } else {
            reprintf(ScreenOutput::ALWAYS, "[s] clinet ip : %s\n", inet_ntoa(clientAddr.sin_addr));
            reprintf(ScreenOutput::ALWAYS, "[s] client accept\n");
        }

		#if 0
        struct timeval time_now{};
        gettimeofday(&time_now, nullptr);
        time_t ts_now;
        ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
		#endif

        uint8_t recvBuffer[BUFSIZ] = {'\0', };
		queTcpRx = std::queue<unsigned char>();

        reprintf(ScreenOutput::NO, "[s] readWriteInfinite while start (%d line)\n", __LINE__);
        while (readWriteInfinite) {
            // reprintf(ScreenOutput::NO, "[s] readWriteInfinite while started (%d line)\n", __LINE__);

			recvLen = recv(clientSock, recvBuffer, BUFSIZ, MSG_PEEK|MSG_DONTWAIT);
			if (recvLen > 0) {
				// 버퍼개수 읽기(MSG_PEEK는 버퍼확인용)
				memset(recvBuffer, '\0', sizeof(recvBuffer));
				recvLen = recv(clientSock, recvBuffer, BUFSIZ, 0);
				for (int i=0; i<recvLen; i++) {
					queTcpRx.push(recvBuffer[i]);
				}
				#if 1
				reprintf(ScreenOutput::NO, "recvLen: %d ", recvLen);
				for (int i=0; i<recvLen; i++) {
					reprintf(ScreenOutput::NO, "[%02x]", recvBuffer[i]);
				}
				reprintf(ScreenOutput::NO, "\n");
				#endif
			} else if (recvLen == 0) {
				// 참고자료
				// recv를 호출했을때 0이 리턴되거나 send를 호출했을때 -1로 에러리턴이면 상대편 연결이 종료된 것

				// 커넥션중일 때는 MSG_PEEK|MSG_DONTWAIT에서 아무것도 송신안할 경우 -1이 발생하였음
				// 커넥션을 강제해제했을 경우 0이 발생하였음
				// MSG_DONTWIT를 추가하지 않았을 경우에는 무한대기가 되었음
				// 우선 이케이스에는 0을 커넥션 종료로 처리함
				reprintf(ScreenOutput::ALWAYS, "[s] client disconnect error, recv: %d \n", recvLen);
				readWriteInfinite = 0;
				continue;
			} else {
                usleep(1000);
			}

			parseTcpState();
        }

        reprintf(ScreenOutput::NO, "[s] tcp read/write end\n");

        ret = close(clientSock);
        reprintf(ScreenOutput::NO, "[s] client socket closed, ret: %d\n", ret);

        ret = close(sock);
        reprintf(ScreenOutput::NO, "[s] socket closed, ret: %d\n", ret);
        // 소켓을 정상적으로 너무 빨리 닫고 재 열기할 경우
        // 해당 포트가 이미 사용중이라는 표시가 나타날 수 있음
        usleep(500000);
    }
}

int main(int argc, char* argv[]) {
    int tcpPort;

    if (argc != ARG_MAX) {
        reprintf(ScreenOutput::ALWAYS, "!!! requirement arguments is %d ea !!!\n", ARG_MAX);
        reprintf(ScreenOutput::ALWAYS, "input argument: %d\n", argc);
        for (int i=0; i<argc; i++) {
            reprintf(ScreenOutput::ALWAYS, "argv[%d]: %s\n", i, argv[i]);
        }

        return -1;
    }
    // argument(host port)
    char* p;
    errno = 0;
    tcpPort = strtol(argv[ARG_TCP_PORT], &p, 10);
    if (*p != '\0' || errno != 0) {
        reprintf(ScreenOutput::ALWAYS, "argument(host port) parsing error.\n");

        return -1;
    }
    reprintf(ScreenOutput::ALWAYS, "argument(host port) is set :[%d].\n", tcpPort);

#define THREAD_TCP_EN 1
	#if THREAD_TCP_EN
	bool isSerial = false;
    boost::thread threadTcp(fThread, &tcpPort, &isSerial);
	#endif

	std::string serialPort = argv[ARG_SERIAL_PORT];
	int baudrate = BAUDRATE;

	ValueInput valueInput, valueInputPre;
	ValueOutput valueOutput, valueOutputPre;

	reprintf(ScreenOutput::ALWAYS, "serial port: %s\n", serialPort.c_str());

	faduino = Faduino(serialPort, baudrate);
	Prog prog;

	if(faduino.initSerial() == false) {
		reprintf(ScreenOutput::ALWAYS, "init() returns false, please check your devices.\n");
		reprintf(ScreenOutput::ALWAYS, "Set port parameters using the following Linux command:\n");
		reprintf(ScreenOutput::ALWAYS, "stty -F /dev/ttyUSB? %d raw\n", baudrate);
		reprintf(ScreenOutput::ALWAYS, "You may need to have ROOT access\n");

		return 0;
	}

	// faduinod와 faduino간 연결 확인후 다음 진행할 것
	// faduino의 ack 명령을 추가할 것
	// sleep는 아래 명령이 빠르게 패스되어 임시로 사용
	sleep(1);

	// PC가 정상적으로 켜졌을 경우 비프 및 LED 상태 알림
	valueOutput.buzzer.onTime = 200;
	valueOutput.buzzer.offTime = 200;
	valueOutput.buzzer.targetCount = STATE_ACT::THRICE;
	valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
	valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
	valueOutput.md_power.onTime = 0;
	valueOutput.md_power.offTime = 0;
	valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
	valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
	valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
	valueOutput.md_estop.onTime = 0;
	valueOutput.md_estop.offTime = 0;
	valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
	valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
	valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
	valueOutput.led_start.onTime = 0;
	valueOutput.led_start.offTime = 0;
	valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
	valueOutput.led_start.lastState = FADUINO::RELAY::OFF;
	valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
	valueOutput.led_stop.onTime = 0;
	valueOutput.led_stop.offTime = 0;
	valueOutput.led_stop.targetCount = STATE_ACT::DIRECT;
	valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
	valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

	valueOutput.buzzer.update = 1;
	valueOutput.md_power.update = 1;
	valueOutput.md_estop.update = 1;
	valueOutput.led_start.update = 1;
	valueOutput.led_stop.update = 1;

	faduino.sendFaduinoCmd(valueOutput);
    
    reprintf(ScreenOutput::ALWAYS, "rosOpenCmd: %s\n", ROS_RUN);
    reprintf(ScreenOutput::ALWAYS, "rosCheckCmd: %s\n", ROS_CHECK);
    reprintf(ScreenOutput::ALWAYS, "rosCloseCmd: %s\n", ROS_KILL);

	#if THREAD_TCP_EN
	isSerial = true;
	#endif

	bool shutdown = false;
	int shutdownFSM = 0;

	while (true) {
		usleep(100);
		faduino.receiveFaduinoState();

		static int sizeFaduinoState;
		sizeFaduinoState = faduino.queFaduinoState.size();

		if (shutdown) {
			switch (shutdownFSM) {
				case 0:
					valueOutput.buzzer.onTime = 1000;
					valueOutput.buzzer.offTime = 1000;
					valueOutput.buzzer.targetCount = STATE_ACT::THRICE;
					valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
					valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
					valueOutput.md_power.onTime = 0;
					valueOutput.md_power.offTime = 0;
					valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
					valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
					valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
					valueOutput.md_estop.onTime = 0;
					valueOutput.md_estop.offTime = 0;
					valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
					valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
					valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
					valueOutput.led_start.onTime = 0;
					valueOutput.led_start.offTime = 0;
					valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
					valueOutput.led_start.lastState = FADUINO::RELAY::OFF;
					valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
					valueOutput.led_stop.onTime = 500;
					valueOutput.led_stop.offTime = 500;
					valueOutput.led_stop.targetCount = STATE_ACT::INFINITE;
					valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
					valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

					valueOutput.buzzer.update = 1;
					valueOutput.md_power.update = 1;
					valueOutput.md_estop.update = 1;
					valueOutput.led_start.update = 1;
					valueOutput.led_stop.update = 1;

					valueOutput.bat_relay.onTime = (valueOutput.buzzer.onTime+valueOutput.buzzer.offTime)*(STATE_ACT::T10);
					valueOutput.bat_relay.offTime = (valueOutput.buzzer.onTime+valueOutput.buzzer.offTime);
					valueOutput.bat_relay.targetCount = STATE_ACT::ONCE;
					valueOutput.bat_relay.lastState = FADUINO::RELAY::OFF;
					valueOutput.bat_relay.order = FADUINO::ORDER::ON_FIRST;

					valueOutput.bat_relay.update = RELAY_MAGIC;

					faduino.sendFaduinoCmd(valueOutput);
					valueOutputPre = valueOutput;

					shutdownFSM++;

					prog.execute(AMR_OFF);
					reprintf(ScreenOutput::NO, "%s\n", AMR_OFF);
					break;
				default:
					break;
			}
		}

		if (sizeFaduinoState) {
			#if 0
			reprintf(ScreenOutput::NO, "faduino.queFaduinoState.size(): %d\n", sizeFaduinoState);
			#endif
			valueInput = faduino.queFaduinoState.front();
			faduino.queFaduinoState.pop();

			#if 0
			reprintf(ScreenOutput::NO, "valueInput   : %1d %1d %1d %1d\nvalueInputPre: %1d %1d %1d %1d\n",
				valueInput.estop_fr, valueInput.estop_bl, valueInput.sw_start, valueInput.sw_stop,
				valueInputPre.estop_fr, valueInputPre.estop_bl, valueInputPre.sw_start, valueInputPre.sw_stop);
			#endif
			
			if (valueInputPre.estop_fr != valueInput.estop_fr ||
				valueInputPre.estop_bl != valueInput.estop_bl ||
				valueInputPre.sw_start != valueInput.sw_start ||
				valueInputPre.sw_stop != valueInput.sw_stop) {
				valueInputPre = valueInput;

				switch (valueInput.estop_fr) {
					case PUSHED:
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.buzzer.onTime = 10000;
						valueOutput.buzzer.offTime = 1000;
						valueOutput.buzzer.targetCount = STATE_ACT::ONCE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_power.onTime = 0;
						valueOutput.md_power.offTime = 0;
						valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_estop.onTime = 0;
						valueOutput.md_estop.offTime = 0;
						valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_start.onTime = 0;
						valueOutput.led_start.offTime = 0;
						valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_start.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_stop.onTime = 1000;
						valueOutput.led_stop.offTime = 1000;
						valueOutput.led_stop.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
						valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

						valueOutput.buzzer.update = 1;
						valueOutput.md_power.update = 1;
						valueOutput.md_estop.update = 1;
						valueOutput.led_start.update = 1;
						valueOutput.led_stop.update = 1;

						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						#if 0
						reprintf(ScreenOutput::NO, "valueInput.estop_fr PUSHED\n");
						#endif
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
				switch (valueInput.estop_bl) {
					case PUSHED:
						#if 1
						// 상위로 전달을 하고 상위에서 수신명령에 대한 처리를 실시
						valueOutput.buzzer.onTime = 10000;
						valueOutput.buzzer.offTime = 1000;
						valueOutput.buzzer.targetCount = STATE_ACT::ONCE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_power.onTime = 0;
						valueOutput.md_power.offTime = 0;
						valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_estop.onTime = 0;
						valueOutput.md_estop.offTime = 0;
						valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_start.onTime = 0;
						valueOutput.led_start.offTime = 0;
						valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_start.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_stop.onTime = 1000;
						valueOutput.led_stop.offTime = 1000;
						valueOutput.led_stop.targetCount = STATE_ACT::INFINITE;
						valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
						valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

						valueOutput.buzzer.update = 1;
						valueOutput.md_power.update = 1;
						valueOutput.md_estop.update = 1;
						valueOutput.led_start.update = 1;
						valueOutput.led_stop.update = 1;

						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						#if 0
						reprintf(ScreenOutput::NO, "valueInput.estop_bl PUSHED\n");
						#endif
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
				switch (valueInput.sw_start) {
					case PUSHED:
						break;
					case RELEASED:
						break;
					case DOUBLE:
						#if 1
						valueOutput.buzzer.onTime = 500;
						valueOutput.buzzer.offTime = 500;
						valueOutput.buzzer.targetCount = STATE_ACT::ONCE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_power.onTime = 0;
						valueOutput.md_power.offTime = 0;
						valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_estop.onTime = 0;
						valueOutput.md_estop.offTime = 0;
						valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_start.onTime = 0;
						valueOutput.led_start.offTime = 0;
						valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_start.lastState = FADUINO::RELAY::ON;
						valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_stop.onTime = 0;
						valueOutput.led_stop.offTime = 0;
						valueOutput.led_stop.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_stop.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

						valueOutput.buzzer.update = 1;
						valueOutput.md_power.update = 1;
						valueOutput.md_estop.update = 1;
						valueOutput.led_start.update = 1;
						valueOutput.led_stop.update = 1;

						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						#if 0
						reprintf(ScreenOutput::NO, "valueInput.sw_start DOUBLE\n");
						#endif

						if (!strlen(prog.execute(ROS_CHECK).c_str())) {
							prog.rosRun();
							reprintf(ScreenOutput::NO, "run ROS\n");
						} else {
							valueOutput.buzzer.onTime = 200;
							valueOutput.buzzer.offTime = 200;
							valueOutput.buzzer.targetCount = STATE_ACT::TWICE;
							valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
							valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
							valueOutput.md_power.onTime = 0;
							valueOutput.md_power.offTime = 0;
							valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
							valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
							valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
							valueOutput.md_estop.onTime = 0;
							valueOutput.md_estop.offTime = 0;
							valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
							valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
							valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
							valueOutput.led_start.onTime = 0;
							valueOutput.led_start.offTime = 0;
							valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
							valueOutput.led_start.lastState = FADUINO::RELAY::ON;
							valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
							valueOutput.led_stop.onTime = 0;
							valueOutput.led_stop.offTime = 0;
							valueOutput.led_stop.targetCount = STATE_ACT::DIRECT;
							valueOutput.led_stop.lastState = FADUINO::RELAY::OFF;
							valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

							valueOutput.buzzer.update = 1;
							valueOutput.md_power.update = 1;
							valueOutput.md_estop.update = 1;
							valueOutput.led_start.update = 1;
							valueOutput.led_stop.update = 1;

							faduino.sendFaduinoCmd(valueOutput);
							valueOutputPre = valueOutput;
							reprintf(ScreenOutput::NO, "already run ROS\n");
						}
						#endif
						break;
					case LONG:
						#if 0
						reprintf(ScreenOutput::NO, "valueInput.sw_start LONG\n");
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
						valueOutput.buzzer.onTime = 500;
						valueOutput.buzzer.offTime = 500;
						valueOutput.buzzer.targetCount = STATE_ACT::TWICE;
						valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
						valueOutput.buzzer.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_power.onTime = 0;
						valueOutput.md_power.offTime = 0;
						valueOutput.md_power.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_power.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_power.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.md_estop.onTime = 0;
						valueOutput.md_estop.offTime = 0;
						valueOutput.md_estop.targetCount = STATE_ACT::DIRECT;
						valueOutput.md_estop.lastState = FADUINO::RELAY::OFF;
						valueOutput.md_estop.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_start.onTime = 0;
						valueOutput.led_start.offTime = 0;
						valueOutput.led_start.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_start.lastState = FADUINO::RELAY::OFF;
						valueOutput.led_start.order = FADUINO::ORDER::ON_FIRST;
						valueOutput.led_stop.onTime = 0;
						valueOutput.led_stop.offTime = 0;
						valueOutput.led_stop.targetCount = STATE_ACT::DIRECT;
						valueOutput.led_stop.lastState = FADUINO::RELAY::ON;
						valueOutput.led_stop.order = FADUINO::ORDER::ON_FIRST;

						valueOutput.buzzer.update = 1;
						valueOutput.md_power.update = 1;
						valueOutput.md_estop.update = 1;
						valueOutput.led_start.update = 1;
						valueOutput.led_stop.update = 1;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						#if 0
						reprintf(ScreenOutput::NO, "valueInput.sw_stop DOUBLE\n");
						#endif

						prog.execute(ROS_KILL);
						reprintf(ScreenOutput::NO, "kill ROS\n");
						#endif
						break;
					case LONG:
						#if 1
						reprintf(ScreenOutput::NO, "valueInput.sw_stop LONG\n");
						#endif
						break;
					case VERYLONG:
						shutdown = true;
						#if 1
						reprintf(ScreenOutput::NO, "valueInput.sw_stop VERYLONG\n");
						#endif
						break;
					case ULTRALONG:
						#if 1
						reprintf(ScreenOutput::NO, "valueInput.sw_stop ULTRALONG\n");
						#endif
						break;
					default:
						break;
				}
			}
		}
	}

	faduino.closeSerial();

	#if THREAD_TCP_EN
    reprintf(ScreenOutput::ALWAYS, "[c] threadTcp join\n");
    threadTcp.join();
    reprintf(ScreenOutput::ALWAYS, "[c] threadTcp joined\n");
	#endif

	return 0;
}