#include <stdio.h>
#include <string>

#include "serial.h"
#include "exec.h"

int main(int argc, char* argv[]) {
	std::string serialPort = argv[1];
	int baudrate = BAUDRATE;

	ValueInput valueInput, valueInputPre;
	ValueOutput valueOutput, valueOutputPre;

	printf("serial port: %s\n", serialPort.c_str());

    boost::thread *threadRos;

	Faduino faduino(serialPort, baudrate);
	Prog prog;

	if(faduino.initSerial() == false) {
		printf("init() returns false, please check your devices.\n");
		printf("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB? %d raw\n", baudrate);
		printf("You may need to have ROOT access\n");

		return 0;
	}

	// faduinod와 faduino간 연결 확인후 다음 진행할 것
	// faduino의 ack 명령을 추가할 것
	// sleep는 아래 명령이 빠르게 패스되어 임시로 사용
	sleep(3);

	// PC가 정상적으로 켜졌을 경우 비프 및 LED 상태 알림
	valueOutput.led_green.onTime = 1000;
	valueOutput.led_green.offTime = 1000;
	valueOutput.led_green.targetCount = STATE_ACT::T10;
	valueOutput.led_green.lastState = FADUINO::RELAY::ON;
	valueOutput.led_red.onTime = 1000;
	valueOutput.led_red.offTime = 1000;
	valueOutput.led_red.targetCount = STATE_ACT::T10;
	valueOutput.led_red.lastState = FADUINO::RELAY::ON;
	valueOutput.buzzer.onTime = 300;
	valueOutput.buzzer.offTime = 200;
	valueOutput.buzzer.targetCount = STATE_ACT::T5;
	valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
	faduino.sendFaduinoCmd(valueOutput);
    
    printf("rosOpenCmd: %s\n", ROS_RUN);
    printf("rosCheckCmd: %s\n", ROS_CHECK);
    printf("rosCloseCmd: %s\n", ROS_KILL);

	while (true) {
		usleep(100);
		faduino.receiveFaduinoState();

		static int sizeFaduinoState;
		sizeFaduinoState = faduino.queFaduinoState.size();
		if (sizeFaduinoState) {
			printf("faduino.queFaduinoState.size(): %d\n", sizeFaduinoState);
			valueInput = faduino.queFaduinoState.front();
			faduino.queFaduinoState.pop();

			printf("valueInput   : %1d %1d %1d %1d\nvalueInputPre: %1d %1d %1d %1d\n",
				valueInput.estop_l, valueInput.estop_r,
				valueInput.sw_green, valueInput.sw_red,
				valueInputPre.estop_l, valueInputPre.estop_r,
				valueInputPre.sw_green, valueInputPre.sw_red);
			
			if (valueInputPre.estop_l != valueInput.estop_l ||
				valueInputPre.estop_r != valueInput.estop_r ||
				valueInputPre.sw_green != valueInput.sw_green ||
				valueInputPre.sw_red != valueInput.sw_red) {
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
							valueOutput.buzzer.targetCount = STATE_ACT::THRICE;
							valueOutput.buzzer.lastState = FADUINO::RELAY::OFF;
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
						faduino.sendFaduinoCmd(valueOutput);
						printf("valueInput.sw_red LONG\n");
						valueOutputPre = valueOutput;
						#endif
						break;
					default:
						break;
				}
			}
		}
	}

	faduino.closeSerial();

    printf("[c] threadRos join\n");
    threadRos->join();
    printf("[c] threadRos joined\n");
//                 threadRos = new boost::thread(exec, rosOpenCmd);
//                 printf("RosOpen\n");
//                 ret = exec(rosCheckCmd);
//                 printf("RosCheck %s\n", ret.c_str());
//                 ret = exec(rosCloseCmd);
//                 printf("RosClose\n");

	return 0;
}