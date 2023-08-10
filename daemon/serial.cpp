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

#include "reprintf.h"
#include "serial.h"

ValueInput valueInput;

#define SERIAL_EN 1

Faduino::Faduino() {
}

Faduino::Faduino(std::string serialPort, int baudrate) {
	this->serialPort = serialPort;
	this->baudrate = baudrate;

	#if SERIAL_EN
	this->ser = new serial::Serial();
	#else
	#endif
}

bool Faduino::initSerial()	{
	#if SERIAL_EN
	const char* COMM_PORT = serialPort.c_str();

	ser->setPort(serialPort);
	ser->setBaudrate(baudrate);
	#define SERIAL_TIMEOUT_MS 3000
	serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
	ser->setTimeout(to);

	ser->open();

	if (!ser->isOpen()) {
		printf("error opening port[%s] baudrate[%d]\n", COMM_PORT, baudrate);
		printf("you may need to have ROOT access\n");
		return false;
	}

	ser->flush();

	reprintf(ScreenOutput::ALWAYS, "faduino communication port is ready\n");
	#else
	reprintf(ScreenOutput::ALWAYS, "SERIAL DUMMY port is ready\n");
	#endif

	return true;
}

void Faduino::closeSerial() {
	#if SERIAL_EN
	ser->close();
	reprintf(ScreenOutput::ALWAYS, "closing faduino\n");
	#else
	reprintf(ScreenOutput::ALWAYS, "closing SERIAL DUMMY\n");
	#endif
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

	#if 0
	reprintf(ScreenOutput::ALWAYS, "SERIAL DATA\n");
	for (int i=0; i<SIZE_TOTAL_OUTPUT; i++) {
		printf("[%02x]", serialBufferTx[i]);
	}
	printf("\n");
	#endif

	#if SERIAL_EN
	ser->write(serialBufferTx, SIZE_TOTAL_OUTPUT);
	#else
	reprintf(ScreenOutput::NO, "SEND SERIAL DUMMY\n");
	reprintf(ScreenOutput::NO, "TS(us): %d\n", *((uint32_t*)(serialBufferTx+IDX_TS)));
	reprintf(ScreenOutput::NO, "buzzer   (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.buzzer.onTime, valueOutput.buzzer.offTime,
		valueOutput.buzzer.targetCount, valueOutput.buzzer.lastState,
		valueOutput.buzzer.update, valueOutput.buzzer.order);
	reprintf(ScreenOutput::NO, "md_power (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.md_power.onTime, valueOutput.md_power.offTime,
		valueOutput.md_power.targetCount, valueOutput.md_power.lastState,
		valueOutput.md_power.update, valueOutput.md_power.order);
	reprintf(ScreenOutput::NO, "md_estop (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.md_estop.onTime, valueOutput.md_estop.offTime,
		valueOutput.md_estop.targetCount, valueOutput.md_estop.lastState,
		valueOutput.md_estop.update, valueOutput.md_estop.order);
	reprintf(ScreenOutput::NO, "led_start(on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.led_start.onTime, valueOutput.led_start.offTime,
		valueOutput.led_start.targetCount, valueOutput.led_start.lastState,
		valueOutput.led_start.update, valueOutput.led_start.order);
	reprintf(ScreenOutput::NO, "led_stop (on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.led_stop.onTime, valueOutput.led_stop.offTime,
		valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState,
		valueOutput.led_stop.update, valueOutput.led_stop.order);
	reprintf(ScreenOutput::NO, "bat_relay(on: %5d off: %5d count: %2d last: %2d update: %2d order: %2d)\n",
		valueOutput.bat_relay.onTime, valueOutput.bat_relay.offTime,
		valueOutput.bat_relay.targetCount, valueOutput.bat_relay.lastState,
		valueOutput.bat_relay.update, valueOutput.bat_relay.order);
	#endif

	return true;
}

bool Faduino::receiveFaduinoState(bool enableParsing) {
	static int rx_size;

	memset(serialBufferRx, '\0', sizeof(serialBufferRx));

	#if SERIAL_EN
	rx_size = ser->available();
	if (rx_size) {
		rx_size = ser->read(serialBufferRx, rx_size);
	}
	#else
	rx_size = 0;
	#endif

	for (int i=0; i<rx_size; i++) {
		queSerialRx.push(serialBufferRx[i]);
	}

	if (enableParsing) {
		parseFaduinoState();
	} else {
		if (queSerialRx.size()) {
			for (int i=0; i<rx_size; i++) {
				reprintf(ScreenOutput::NO, "[%02x]", queSerialRx.front());
				queSerialRx.pop();
			}
			reprintf(ScreenOutput::NO, "\n");
		}
	}

	return true;
}

bool Faduino::parseFaduinoState() {
	static int state = FSM_FADUINO::HEAD;
	static unsigned char packet[SIZE_TOTAL_INPUT] = {'\0', };

	switch (state) {
		case FSM_FADUINO::HEAD:
			if (queSerialRx.size() >= SIZE_HEAD) {
				packet[IDX_HEAD] = queSerialRx.front();
				if (packet[IDX_HEAD] == DATA_HEAD) {
					state = FSM_FADUINO::TYPE;
				} else {
					reprintf(ScreenOutput::NO, "FSM_FADUINO::HEAD not Match \n");
					state = FSM_FADUINO::HEAD;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_FADUINO::TYPE:
			if (queSerialRx.size() >= SIZE_TYPE) {
				packet[IDX_TYPE] = queSerialRx.front();
				state = FSM_FADUINO::TS;
				queSerialRx.pop();
			}
			break;
		case FSM_FADUINO::TS:
			if (queSerialRx.size() >= SIZE_TS) {
				for (int i=0; i<SIZE_TS; i++) {
					packet[IDX_TS+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_FADUINO::DATA;
			}
			break;
		case FSM_FADUINO::DATA:
			if (queSerialRx.size() >= SIZE_DATA_INPUT) {
				for (int i=0; i<SIZE_DATA_INPUT; i++) {
					packet[IDX_DATA+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_FADUINO::CRC16;
			}
			break;
		case FSM_FADUINO::CRC16:
			if (queSerialRx.size() >= SIZE_CRC16) {
				for (int i=0; i<SIZE_CRC16; i++) {
					packet[IDX_CRC16_INPUT+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_FADUINO::TAIL;
			}
			break;
		case FSM_FADUINO::TAIL:
			if (queSerialRx.size() >= SIZE_TAIL) {
				packet[IDX_TAIL_INPUT] = queSerialRx.front();
				if (packet[IDX_TAIL_INPUT] == DATA_TAIL) {
					state = FSM_FADUINO::OK;
				} else {
					reprintf(ScreenOutput::NO, "FSM_FADUINO::TAIL not Match\n");
					for (int i=0; i<SIZE_TOTAL_INPUT; i++) {
						reprintf(ScreenOutput::NO, "[%02x]", packet[i]);
					}
					reprintf(ScreenOutput::NO, "\n");
					state = FSM_FADUINO::HEAD;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_FADUINO::OK:
			checksumFaduinoState(packet);

			memset(packet, '\0', SIZE_TOTAL_INPUT);
			
			state = FSM_FADUINO::HEAD;

			break;
		default:
			state = FSM_FADUINO::HEAD;

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
				sscanf((const char*)(packet+IDX_DATA), "%01hd%01hd%01hd%01hd%01hd%01hd",
					&valueInput.estop_fr, &valueInput.estop_bl, &valueInput.sw_start, &valueInput.sw_stop, &valueInput.in_spare1, &valueInput.in_spare2);

				queFaduinoState.push(valueInput);

				#if 0
				reprintf(ScreenOutput::NO, "type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				switch (valueInput.estop_fr) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
					default:
						break;
				}
				switch (valueInput.estop_bl) {
					case PUSHED:
					case RELEASED:
					case DOUBLE:
					case LONG:
					default:
						break;
				}
				switch (valueInput.sw_start) {
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
				reprintf(ScreenOutput::NO, "type:%d, ",
					packet[IDX_TYPE]);
				reprintf(ScreenOutput::NO, "estop_fr:%d, estop_bl:%d, sw_start:%d, sw_stop:%d, in_spare1:%d, in_spare2:%d, ts:%d\n",
					valueInput.estop_fr, valueInput.estop_bl, valueInput.sw_start, valueInput.sw_stop, valueInput.in_spare1, valueInput.in_spare2, *((int*)(packet+IDX_TS)));
				#endif
				break;
			case TYPE_CMD::SENSOR:
				static int sensorValue;
				sscanf((const char*)(packet+IDX_DATA), "%05d", &sensorValue);

				#if 1
				reprintf(ScreenOutput::NO, "type:%d, ts:%d, ", packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				reprintf(ScreenOutput::NO, "sensorValue:%d\n", sensorValue);
				#endif
				break;
			default:
				reprintf(ScreenOutput::NO, "unknown type:%d, ts:%d\n",
					packet[IDX_TYPE], *((int*)(packet+IDX_TS)));
				break;
		}
	} else {
		reprintf(ScreenOutput::NO, "crc16 not matched !!!\n");
	}
}