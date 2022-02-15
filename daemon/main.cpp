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
    
    printf("rosOpenCmd: %s\n", ROS_RUN);
    printf("rosCheckCmd: %s\n", ROS_CHECK);
    printf("rosCloseCmd: %s\n", ROS_KILL);

	while (true) {
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
						valueOutput.led_green.on = 0;
						valueOutput.led_green.off = 0;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 1000;
						valueOutput.led_red.off = 0;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 1000;
						valueOutput.buzzer.off = 0;
						valueOutput.buzzer.act = STATE_ACT::INFINITE;
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
						valueOutput.led_green.on = 0;
						valueOutput.led_green.off = 0;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 1000;
						valueOutput.led_red.off = 0;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 1000;
						valueOutput.buzzer.off = 0;
						valueOutput.buzzer.act = STATE_ACT::INFINITE;
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
						valueOutput.led_green.on = 1000;
						valueOutput.led_green.off = 0;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 0;
						valueOutput.led_red.off = 1000;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 500;
						valueOutput.buzzer.off = 500;
						valueOutput.buzzer.act = STATE_ACT::ONCE;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						printf("valueInput.sw_green DOUBLE\n");

						if (!strlen(prog.execute(ROS_CHECK).c_str())) {
							prog.rosRun();
							printf("run ROS\n");
						} else {
							valueOutput.led_green.on = 1000;
							valueOutput.led_green.off = 0;
							valueOutput.led_green.act = STATE_ACT::INFINITE;
							valueOutput.led_red.on = 0;
							valueOutput.led_red.off = 1000;
							valueOutput.led_red.act = STATE_ACT::INFINITE;
							valueOutput.buzzer.on = 300;
							valueOutput.buzzer.off = 200;
							valueOutput.buzzer.act = STATE_ACT::THRICE;
							faduino.sendFaduinoCmd(valueOutput);
							printf("already run ROS\n");
						}
						#endif
						break;
					case LONG:
						#if 1
						valueOutput.led_green.on = 0;
						valueOutput.led_green.off = 1000;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 0;
						valueOutput.led_red.off = 1000;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 0;
						valueOutput.buzzer.off = 1000;
						valueOutput.buzzer.act = STATE_ACT::INFINITE;
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
						valueOutput.led_green.on = 0;
						valueOutput.led_green.off = 1000;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 1000;
						valueOutput.led_red.off = 0;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 500;
						valueOutput.buzzer.off = 500;
						valueOutput.buzzer.act = STATE_ACT::TWICE;
						faduino.sendFaduinoCmd(valueOutput);
						valueOutputPre = valueOutput;
						printf("valueInput.sw_red DOUBLE\n");

						prog.execute(ROS_KILL);
						printf("kill ROS\n");
						#endif
						break;
					case LONG:
						#if 1
						valueOutput.led_green.on = 0;
						valueOutput.led_green.off = 1000;
						valueOutput.led_green.act = STATE_ACT::INFINITE;
						valueOutput.led_red.on = 0;
						valueOutput.led_red.off = 1000;
						valueOutput.led_red.act = STATE_ACT::INFINITE;
						valueOutput.buzzer.on = 0;
						valueOutput.buzzer.off = 1000;
						valueOutput.buzzer.act = STATE_ACT::INFINITE;
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