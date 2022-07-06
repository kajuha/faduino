#include "PinButton.h"
#include "Flasher.h"

#include "protocol_serial.h"

// 출력(릴레이)핀 및 종류
#define PIN_OUTPUT_SW_PC 22
#define PIN_OUTPUT_BUZZER 23 // wire(white)
#define PIN_OUTPUT_LED_GREEN 24 // wire(yellow)
#define PIN_OUTPUT_LED_RED 25 // wire(purple)
#define PIN_OUTPUT_LED_START 26
#define PIN_OUTPUT_LED_STOP 27
#define PIN_OUTPUT_REL_BREAK 28

// 입력핀 및 종류
#define PIN_INPUT_ESTOP_L 30 // wire(black)
#define PIN_INPUT_ESTOP_R 31 // wire(green)
#define PIN_INPUT_SW_GREEN 32 // wire(gray)
#define PIN_INPUT_SW_RED 33 // wire(brown)
#define PIN_INPUT_SW_STOP 34 // NC(normal close) 버튼
#define SW_STOP_ERROR 0 // 측정시 0.2V의 플로팅된 전압이 생겼음, 이로인해 부팅시 STOP 버튼이 동작됨

#define PIN_INPUT_CURRENT A4

#define CURRENT_VOLTAGE 5.0
#define MAX_VOLTAGE 10.0
#define MAX_STEP 1024.0

// 상위 제어기에서 송수신되는 입출력값 및 이전값
ValueInput valueInput, valueInputBefore;
ValueOutput valueOutput, valueOutputBefore;

// 시간관련 변수
// 상위제어기 부팅시작 시간
unsigned long ts_bootup_start;
// 현재 시간
unsigned long ts_now;

// 입력핀 변수(클래스)(더블클릭 및 롱클릭 등 감지)
PinButton swGreen(PIN_INPUT_SW_GREEN, INPUT);
PinButton swRed(PIN_INPUT_SW_RED, INPUT);
#if SW_STOP_ERROR
PinButton swStop(PIN_INPUT_SW_STOP, INPUT_PULLUP);
#endif

// 출력핀 변수(클래스)(블링크 및 온오프 등 출력)
Flasher swPc(PIN_OUTPUT_SW_PC, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher buzzer(PIN_OUTPUT_BUZZER, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledStart(PIN_OUTPUT_LED_START, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledGreen(PIN_OUTPUT_LED_GREEN, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledStop(PIN_OUTPUT_LED_STOP, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledRed(PIN_OUTPUT_LED_RED, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher relBreak(PIN_OUTPUT_REL_BREAK, 0, 0, INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);

int stateEstopL;
int stateEstopR;

int currentValue;
static char buffer[BUFSIZ];

unsigned long us_now, us_pre;

void setup() {
  // 통신 설정
  Serial1.begin(BAUDRATE);

  // 입력핀(비상스위치) 설정
  pinMode(PIN_INPUT_ESTOP_L, INPUT);
  pinMode(PIN_INPUT_ESTOP_R, INPUT);
  swGreen.update();
  #if SW_STOP_ERROR
  swStop.update();
  #endif
  swRed.update();

  us_now = us_pre = millis();
  
  // 출력핀 설정
  swPc.setOnOffTime(BOOT_SW_MSEC, 1000, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  buzzer.setOnOffTime(200, 200, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledStart.setOnOffTime(500, 500, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledGreen.setOnOffTime(500, 500, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledStop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledRed.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  relBreak.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
  relBreak.update();
  ts_now = millis();
  ts_bootup_start = ts_now;
}

void loop() {
  // 현재 시간 검사
  ts_now = millis();

  // 입력핀 상태(더블클릭, 롱클릭 등) 처리
  swGreen.update();
  #if SW_STOP_ERROR
  swStop.update();
  #endif
  swRed.update();
  currentValue = analogRead(PIN_INPUT_CURRENT);

  // 어떤 클릭인지 확인
  if (swGreen.isDoubleClick()) {
    valueInput.sw_green = STATE_INPUT::DOUBLE;
  } else if (swGreen.isLongClick()) {
    valueInput.sw_green = STATE_INPUT::LONG;
  } else {
    valueInput.sw_green = STATE_INPUT::RELEASED;
  }
  
  // 어떤 클릭인지 확인
  #if SW_STOP_ERROR
  if (swStop.isDoubleClick()) {
    valueInput.sw_stop = STATE_INPUT::DOUBLE;
  } else if (swStop.isLongClick()) {
    valueInput.sw_stop = STATE_INPUT::LONG;

    buzzer.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledGreen.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
    ledRed.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
  } else if (swStop.isVeryLongClick()) {
    valueInput.sw_stop = STATE_INPUT::VERYLONG;
  } else if (swStop.isUltraLongClick()) {
    valueInput.sw_stop = STATE_INPUT::ULTRALONG;

    buzzer.setOnOffTime(500, 500, STATE_ACT::T5, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledGreen.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledRed.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);

    relBreak.setOnOffTime((500+500)*(5+1), 1000, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  } else {
    valueInput.sw_stop = STATE_INPUT::RELEASED;
  }
  #endif
  
  // 어떤 클릭인지 확인
  if (swRed.isDoubleClick()) {
    valueInput.sw_red = STATE_INPUT::DOUBLE;
  } else if (swRed.isLongClick()) {
    valueInput.sw_red = STATE_INPUT::LONG;

    buzzer.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledGreen.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
    ledRed.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
  } else if (swRed.isVeryLongClick()) {
    valueInput.sw_red = STATE_INPUT::VERYLONG;
  } else if (swRed.isUltraLongClick()) {
    valueInput.sw_red = STATE_INPUT::ULTRALONG;

    buzzer.setOnOffTime(500, 500, STATE_ACT::T5, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledGreen.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledRed.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);

    relBreak.setOnOffTime((500+500)*(5+1), 1000, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  } else {
    valueInput.sw_red = STATE_INPUT::RELEASED;
  }
  
  // 비상스위치 눌렸는지 확인
  if (digitalRead(PIN_INPUT_ESTOP_L)) {
    switch (stateEstopL) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_l = STATE_INPUT::PUSHED;
        stateEstopL = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  } else {
    valueInput.estop_l = STATE_INPUT::RELEASED;
    stateEstopL = STATE_ESTOP::ACTION;
  }
  
  // 비상스위치 눌렸는지 확인
  if (digitalRead(PIN_INPUT_ESTOP_R)) {
    switch (stateEstopR) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_r = STATE_INPUT::PUSHED;
        stateEstopR = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  } else {
    valueInput.estop_r = STATE_INPUT::RELEASED;
    stateEstopR = STATE_ESTOP::ACTION;
  }

  // 입력핀에 대한 처리
  if (valueInput.sw_green == STATE_INPUT::LONG) {
    // 부팅된 상태인가?
    if (false) {
      // 아무것도 하지 않음
      // 이미 부팅된 상태를 알려줄 것
    }
    // 부팅시간을 초과하지 않았는가?
    else if ((ts_now - ts_bootup_start) > BOOTUP_MSEC_PC) {
      // 아무것도 하지 않음
      // 부팅시간이 초과되지 않음을 알려줄 것
    }
    // 부팅된 상태도 아니고 부팅
    else {
      swPc.setOnOffTime(BOOT_SW_MSEC, 1000, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
      ts_bootup_start = ts_now;
    }
  }

  // 송신 데이터 처리(이전과 상태변화가 달라졌으면 입력핀 상태 송신)
  if (valueInputBefore.sw_green != valueInput.sw_green ||
      valueInputBefore.sw_stop != valueInput.sw_stop ||
      valueInputBefore.sw_red != valueInput.sw_red ||
      valueInputBefore.estop_l != valueInput.estop_l ||
      valueInputBefore.estop_r != valueInput.estop_r) {
    valueInputBefore = valueInput;

    // 송신 포맷 생성
    buffer[IDX_HEAD] = DATA_HEAD;
    buffer[IDX_TYPE] = TYPE_CMD::CMD;
    *((unsigned long*)(buffer+IDX_TS)) = micros();
    sprintf(buffer+IDX_DATA, "%01d%01d%01d%01d%01d", valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red, valueInput.sw_stop);
    // crc16 계산
    unsigned short crc16in = CRC::CRC16((unsigned char*)(buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);
    sprintf(buffer+IDX_CRC16_INPUT, "%04x", crc16in);
    buffer[IDX_TAIL_INPUT] = DATA_TAIL;

    Serial1.write(buffer, SIZE_TOTAL_INPUT);
  }

  // 주기마다 아날로그 데이터 송신
  us_now = micros();
  #define SEND_RATE_US  1000000
  // FADUINO에서 프로그래밍(펌웨어 다운로드) 포트로
  // 일반 송수신 통신을 할 경우 에러가 발생함
  // 일반 송수신은 다른 포트를 이용할 것
  #if 0
  if ((us_now - us_pre) > SEND_RATE_US) {
    us_pre = us_now;

    // 송신 포맷 생성
    buffer[IDX_HEAD] = DATA_HEAD;
    buffer[IDX_TYPE] = TYPE_CMD::SENSOR;
    *((unsigned long*)(buffer+IDX_TS)) = micros();
    sprintf(buffer+IDX_DATA, "%05d", currentValue);
    // crc16 계산
    unsigned short crc16in = CRC::CRC16((unsigned char*)(buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);
    sprintf(buffer+IDX_CRC16_INPUT, "%04x", crc16in);
    buffer[IDX_TAIL_INPUT] = DATA_TAIL;

    Serial1.write(buffer, SIZE_TOTAL_INPUT);
  }
  #endif
  // 수신 데이터 처리
  //   가. 송신 데이터에 대한 ACK
  //   나. 수신 데이터(출력)에 대한 현재 상태 요청에 대한 응답
  //   다. 수신 데이터(출력)에 대한 설정
  parseData();
  // 출력 처리
  swPc.update();
  buzzer.update();
  ledStart.update();
  ledGreen.update();
  ledStop.update();
  ledRed.update();
  relBreak.update();
}

bool parseData() {
  static int state = FSM_FADUINO::HEAD;
  static unsigned char packet[SIZE_TOTAL_OUTPUT] = {'\0', };

  switch (state) {
    case FSM_FADUINO::HEAD:
      if (Serial1.available() >= SIZE_HEAD) {
        #if 1
        Serial1.readBytes(packet+IDX_HEAD, SIZE_HEAD);
        #else
        packet[IDX_HEAD] = que.front();
        que.pop();
        #endif
        if (packet[IDX_HEAD] == DATA_HEAD) {
          state = FSM_FADUINO::TYPE;
        } else {
          state = FSM_FADUINO::HEAD;
        }
      }
      break;
    case FSM_FADUINO::TYPE:
      if (Serial1.available() >= SIZE_TYPE) {
        #if 1
        Serial1.readBytes(packet+IDX_TYPE, SIZE_TYPE);
        #else
        packet[IDX_TYPE] = que.front();
        que.pop();
        #endif
        
        state = FSM_FADUINO::TS;
      }
      break;
    case FSM_FADUINO::TS:
      if (Serial1.available() >= SIZE_TS) {
        #if 1
        Serial1.readBytes(packet+IDX_TS, SIZE_TS);
        #else
        for (int i=0; i<SIZE_TS; i++) {
          packet[IDX_TS+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_FADUINO::DATA;
      }
      break;
    case FSM_FADUINO::DATA:
      if (Serial1.available() >= SIZE_DATA_OUTPUT) {
        #if 1
        Serial1.readBytes(packet+IDX_DATA, SIZE_DATA_OUTPUT);
        #else
        for (int i=0; i<SIZE_DATA_OUTPUT; i++) {
          packet[IDX_DATA+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_FADUINO::CRC16;
      }
      break;
    case FSM_FADUINO::CRC16:
      if (Serial1.available() >= SIZE_CRC16) {
        #if 1
        Serial1.readBytes(packet+IDX_CRC16_OUTPUT, SIZE_CRC16);
        #else
        for (int i=0; i<SIZE_CRC16; i++) {
          packet[IDX_CRC16+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_FADUINO::TAIL;
      }
      break;
    case FSM_FADUINO::TAIL:
      if (Serial1.available() >= SIZE_TAIL) {
        #if 1
        Serial1.readBytes(packet+IDX_TAIL_OUTPUT, SIZE_TAIL);
        #else
        packet[IDX_TAIL] = que.front();
        que.pop();
        #endif
        if (packet[IDX_TAIL_OUTPUT] == DATA_TAIL) {
          state = FSM_FADUINO::OK;
        } else {
          state = FSM_FADUINO::HEAD;
        }
      }
      break;
    case FSM_FADUINO::OK:
      checksumData(packet);

      memset(packet, '\0', SIZE_TOTAL_OUTPUT);
      
      state = FSM_FADUINO::HEAD;

      break;
    default:
      state = FSM_FADUINO::HEAD;

      break;
  }
  
  return false;
}

void checksumData(unsigned char* packet)
{
  // 수신부 crc16 문자열 추출
  unsigned short crc16out;
  sscanf((const char*)(packet+IDX_CRC16_OUTPUT), "%04x", (unsigned int*)&crc16out);

  // 수신부 data의 crc16 계산
  unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);

  if (crc16out == crc16) {
    switch (packet[IDX_TYPE]) {
      case TYPE_CMD::CMD:
        memcpy(&valueOutput, packet+IDX_DATA, SIZE_DATA_OUTPUT);
        if (valueOutput.buzzer.update) buzzer.setOnOffTime(valueOutput.buzzer.onTime, valueOutput.buzzer.offTime, valueOutput.buzzer.targetCount, valueOutput.buzzer.lastState, valueOutput.buzzer.order);
        if (valueOutput.led_start.update) ledStart.setOnOffTime(valueOutput.led_start.onTime, valueOutput.led_start.offTime, valueOutput.led_start.targetCount, valueOutput.led_start.lastState, valueOutput.led_start.order);
        if (valueOutput.led_green.update) ledGreen.setOnOffTime(valueOutput.led_green.onTime, valueOutput.led_green.offTime, valueOutput.led_green.targetCount, valueOutput.led_green.lastState, valueOutput.led_green.order);
        if (valueOutput.led_stop.update) ledStop.setOnOffTime(valueOutput.led_stop.onTime, valueOutput.led_stop.offTime, valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState, valueOutput.led_stop.order);
        if (valueOutput.led_red.update) ledRed.setOnOffTime(valueOutput.led_red.onTime, valueOutput.led_red.offTime, valueOutput.led_red.targetCount, valueOutput.led_red.lastState, valueOutput.led_red.order);
        if (valueOutput.rel_break.update == BREAK_MAGIC) relBreak.setOnOffTime(valueOutput.rel_break.onTime, valueOutput.rel_break.offTime, valueOutput.rel_break.targetCount, valueOutput.rel_break.lastState, valueOutput.rel_break.order);
        break;
      case TYPE_CMD::HB:
        break;
      default:
        break;
    }
  } else {
  }
}