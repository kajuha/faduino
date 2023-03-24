#include "PinButton.h"
#include "Flasher.h"

#include "protocol_serial.h"

// 출력(릴레이)핀 및 종류
#define PIN_OUTPUT_SW_PC 29
#define PIN_OUTPUT_BUZZER 37
#define PIN_OUTPUT_MD_POWER 33
#define PIN_OUTPUT_MD_ESTOP 32
#define PIN_OUTPUT_LED_START 36
#define PIN_OUTPUT_LED_STOP 35
#define PIN_OUTPUT_BAT_RELAY 34
#define PIN_OUTPUT_SPARE2 31
#define PIN_OUTPUT_SPARE1 30

// 입력핀 및 종류
#define PIN_INPUT_ESTOP_FR 23
#define PIN_INPUT_ESTOP_BL 22
#define PIN_INPUT_SW_START 26
#define PIN_INPUT_SW_STOP 27
#define PIN_INPUT_SPARE1 24
#define PIN_INPUT_SPARE2 25


#define PIN_OUTPUT_RS485_EN 6

// 상위 제어기에서 송수신되는 입출력값 및 이전값
ValueInput valueInput, valueInputBefore;
ValueOutput valueOutput, valueOutputBefore;

// 시간관련 변수
// 상위제어기 부팅시작 시간
unsigned long ts_bootup_start;
// 현재 시간
unsigned long ts_now;

// 입력핀 변수(클래스)(더블클릭 및 롱클릭 등 감지)
PinButton swStart(PIN_INPUT_SW_START, INPUT);
PinButton swStop(PIN_INPUT_SW_STOP, INPUT);

// 출력핀 변수(클래스)(블링크 및 온오프 등 출력)
Flasher swPc(PIN_OUTPUT_SW_PC, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher buzzer(PIN_OUTPUT_BUZZER, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher mdPower(PIN_OUTPUT_MD_POWER, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher mdEstop(PIN_OUTPUT_MD_ESTOP, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledStart(PIN_OUTPUT_LED_START, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher ledStop(PIN_OUTPUT_LED_STOP, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher batRelay(PIN_OUTPUT_BAT_RELAY, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher outSpare1(PIN_OUTPUT_SPARE1, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
Flasher outSpare2(PIN_OUTPUT_SPARE2, 0, 0, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);

int stateEstopFR;
int stateEstopBL;

static char buffer[BUFSIZ];

unsigned long us_now, us_pre;

void setup() {
  // 통신 설정
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE);

  // RS485 출력핀 LOW로 설정
  pinMode(PIN_OUTPUT_RS485_EN, OUTPUT);
  digitalWrite(PIN_OUTPUT_RS485_EN, 0);

  // 입력핀(비상스위치) 설정
  pinMode(PIN_INPUT_ESTOP_FR, INPUT_PULLUP);
  pinMode(PIN_INPUT_ESTOP_BL, INPUT_PULLUP);
  swStart.update();
  swStop.update();

  us_now = us_pre = millis();
  
  // 출력핀 설정
  swPc.setOnOffTime(BOOT_SW_MSEC, 1000, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  buzzer.setOnOffTime(200, 200, STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  mdPower.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  mdEstop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledStart.setOnOffTime(500, 500, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ledStop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  batRelay.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
  batRelay.update();
  outSpare1.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  outSpare2.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  ts_now = millis();
  ts_bootup_start = ts_now;
}

void loop() {
  // 현재 시간 검사
  ts_now = millis();

  // 입력핀 상태(더블클릭, 롱클릭 등) 처리
  swStart.update();
  swStop.update();

  // 어떤 클릭인지 확인
  if (swStart.isDoubleClick()) {
    valueInput.sw_start = STATE_INPUT::DOUBLE;
  } else if (swStart.isLongClick()) {
    valueInput.sw_start = STATE_INPUT::LONG;
  } else {
    valueInput.sw_start = STATE_INPUT::RELEASED;
  }
  
  // 어떤 클릭인지 확인
  if (swStop.isDoubleClick()) {
    valueInput.sw_stop = STATE_INPUT::DOUBLE;
  } else if (swStop.isLongClick()) {
    valueInput.sw_stop = STATE_INPUT::LONG;

    buzzer.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::ON, FADUINO::ORDER::ON_FIRST);
  } else if (swStop.isVeryLongClick()) {
    valueInput.sw_stop = STATE_INPUT::VERYLONG;
  } else if (swStop.isUltraLongClick()) {
    valueInput.sw_stop = STATE_INPUT::ULTRALONG;

    TimeOnOff buzzerTime;
    buzzerTime.onTime = 500;
    buzzerTime.offTime = 500;
    buzzerTime.targetCount = STATE_ACT::T5;
    buzzer.setOnOffTime(buzzerTime.onTime, buzzerTime.offTime, buzzerTime.targetCount, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    mdPower.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    mdEstop.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStart.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    ledStop.setOnOffTime(1000, 1000, STATE_ACT::INFINITE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    outSpare1.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
    outSpare2.setOnOffTime(0, 0, STATE_ACT::DIRECT, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);

    batRelay.setOnOffTime((buzzerTime.onTime+buzzerTime.offTime)*(buzzerTime.targetCount+1), (buzzerTime.onTime+buzzerTime.offTime), STATE_ACT::ONCE, FADUINO::RELAY::OFF, FADUINO::ORDER::ON_FIRST);
  } else {
    valueInput.sw_stop = STATE_INPUT::RELEASED;
  }
  
  // 비상스위치 눌렸는지 확인
  if (digitalRead(PIN_INPUT_ESTOP_FR)) {
    valueInput.estop_fr = STATE_INPUT::RELEASED;
    stateEstopFR = STATE_ESTOP::ACTION;
  } else {
    switch (stateEstopFR) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_fr = STATE_INPUT::PUSHED;
        stateEstopFR = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  }
  
  // 비상스위치 눌렸는지 확인
  if (digitalRead(PIN_INPUT_ESTOP_BL)) {
    valueInput.estop_bl = STATE_INPUT::RELEASED;
    stateEstopBL = STATE_ESTOP::ACTION;
  } else {
    switch (stateEstopBL) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_bl = STATE_INPUT::PUSHED;
        stateEstopBL = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  }

  // 입력핀에 대한 처리
  if (valueInput.sw_start == STATE_INPUT::LONG) {
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
  if (valueInputBefore.estop_fr != valueInput.estop_fr ||
      valueInputBefore.estop_bl != valueInput.estop_bl ||
      valueInputBefore.sw_start != valueInput.sw_start ||
      valueInputBefore.sw_stop != valueInput.sw_stop) {
    valueInputBefore = valueInput;

    // 송신 포맷 생성
    buffer[IDX_HEAD] = DATA_HEAD;
    buffer[IDX_TYPE] = TYPE_CMD::CMD;
    *((unsigned long*)(buffer+IDX_TS)) = micros();
    sprintf(buffer+IDX_DATA, "%01d%01d%01d%01d", valueInput.estop_fr, valueInput.estop_bl, valueInput.sw_start, valueInput.sw_stop);
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
    buffer[IDX_TYPE] = TYPE_CMD::HB;
    *((unsigned long*)(buffer+IDX_TS)) = micros();
    int dummy = 0;
    sprintf(buffer+IDX_DATA, "%04d", dummy);
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
  mdPower.update();
  mdEstop.update();
  ledStart.update();
  ledStop.update();
  batRelay.update();
  outSpare1.update();
  outSpare2.update();
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
    #if 0
    case FSM_FADUINO::DATA:
      #if 0
      // 아두이노의 시리얼 수신 버퍼의 크기가 64바이트임
      // 64바이트를 초과할 경우 데이터 수신불가
      Serial.println(Serial1.available());
      #endif
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
    #else
    case FSM_FADUINO::DATA:
      #if 0
      Serial.println(Serial1.available());
      #endif

      static FSM_ARDUINO_SERIAL_RX fsmArduinoSerialRx;
      static int bufferedByteSize;
      static int dataByteSizeToRead;

      switch (fsmArduinoSerialRx) {
        case FSM_ARDUINO_SERIAL_RX::INIT:
          dataByteSizeToRead = SIZE_DATA_OUTPUT;
          fsmArduinoSerialRx = FSM_ARDUINO_SERIAL_RX::READ;
          break;
        case FSM_ARDUINO_SERIAL_RX::READ:
          bufferedByteSize = Serial1.available();
          if ((bufferedByteSize-dataByteSizeToRead) >= 0) {
            Serial1.readBytes(packet+IDX_DATA+(SIZE_DATA_OUTPUT-dataByteSizeToRead), dataByteSizeToRead);
            fsmArduinoSerialRx = FSM_ARDUINO_SERIAL_RX::INIT;
            state = FSM_FADUINO::CRC16;
          } else {
            Serial1.readBytes(packet+IDX_DATA+(SIZE_DATA_OUTPUT-dataByteSizeToRead), bufferedByteSize);
            dataByteSizeToRead -= bufferedByteSize;
          }
          break;
        default:
          Serial.println("IMPOSSIBLE CASE");
          break;
      }
      break;
    #endif
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
        if (valueOutput.md_power.update == MD_MAGIC) mdPower.setOnOffTime(valueOutput.md_power.onTime, valueOutput.md_power.offTime, valueOutput.md_power.targetCount, valueOutput.md_power.lastState, valueOutput.md_power.order);
        if (valueOutput.md_estop.update == MD_MAGIC) mdEstop.setOnOffTime(valueOutput.md_estop.onTime, valueOutput.md_estop.offTime, valueOutput.md_estop.targetCount, valueOutput.md_estop.lastState, valueOutput.md_estop.order);
        if (valueOutput.led_start.update) ledStart.setOnOffTime(valueOutput.led_start.onTime, valueOutput.led_start.offTime, valueOutput.led_start.targetCount, valueOutput.led_start.lastState, valueOutput.led_start.order);
        if (valueOutput.led_stop.update) ledStop.setOnOffTime(valueOutput.led_stop.onTime, valueOutput.led_stop.offTime, valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState, valueOutput.led_stop.order);
        if (valueOutput.bat_relay.update == RELAY_MAGIC) batRelay.setOnOffTime(valueOutput.bat_relay.onTime, valueOutput.bat_relay.offTime, valueOutput.bat_relay.targetCount, valueOutput.bat_relay.lastState, valueOutput.bat_relay.order);
        if (valueOutput.out_spare1.update) outSpare1.setOnOffTime(valueOutput.out_spare1.onTime, valueOutput.out_spare1.offTime, valueOutput.out_spare1.targetCount, valueOutput.out_spare1.lastState, valueOutput.out_spare1.order);
        if (valueOutput.out_spare2.update) outSpare2.setOnOffTime(valueOutput.out_spare2.onTime, valueOutput.out_spare2.offTime, valueOutput.out_spare2.targetCount, valueOutput.out_spare2.lastState, valueOutput.out_spare2.order);
        break;
      case TYPE_CMD::HB:
        break;
      default:
        break;
    }
  } else {
  }
}