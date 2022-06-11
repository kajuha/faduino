#include "PinButton.h"
#include "Flasher.h"

#include "protocol_serial.h"

#define PIN_OUTPUT_SW_PC 22
#define PIN_OUTPUT_BUZZER 23 // wire(white)
#define PIN_OUTPUT_LED_GREEN 24 // wire(yellow)
#define PIN_OUTPUT_LED_RED 25 // wire(purple)
#define PIN_OUTPUT_LED_START 26
#define PIN_OUTPUT_LED_STOP 27
#define PIN_OUTPUT_REL_BREAK 28

#define PIN_INPUT_ESTOP_L 30 // wire(black)
#define PIN_INPUT_ESTOP_R 31 // wire(green)
#define PIN_INPUT_SW_GREEN 32 // wire(gray)
#define PIN_INPUT_SW_RED 33 // wire(brown)
#define PIN_INPUT_SW_STOP 34 // NC(normal close) 버튼

#define PIN_INPUT_CURRENT A4

#define CURRENT_VOLTAGE 5.0
#define MAX_VOLTAGE 10.0
#define MAX_STEP 1024.0

ValueInput valueInput, valueInputBefore;
ValueOutput valueOutput, valueOutputBefore;

PinButton swGreen(PIN_INPUT_SW_GREEN, INPUT);
PinButton swRed(PIN_INPUT_SW_RED, INPUT);
PinButton swStop(PIN_INPUT_SW_STOP, INPUT_PULLUP);

Flasher buzzer(PIN_OUTPUT_BUZZER, 0, 0, INFINITE, FADUINO::RELAY::OFF);
Flasher ledGreen(PIN_OUTPUT_LED_GREEN, 0, 0, INFINITE, FADUINO::RELAY::OFF);
Flasher ledRed(PIN_OUTPUT_LED_RED, 0, 0, INFINITE, FADUINO::RELAY::OFF);
Flasher ledStart(PIN_OUTPUT_LED_START, 0, 0, INFINITE, FADUINO::RELAY::OFF);
Flasher ledStop(PIN_OUTPUT_LED_STOP, 0, 0, INFINITE, FADUINO::RELAY::OFF);
Flasher relBreak(PIN_OUTPUT_REL_BREAK, 0, 0, INFINITE, FADUINO::RELAY::OFF);

int stateEstopL;
int stateEstopR;

int currentValue;
static char buffer[BUFSIZ];

unsigned long us_now, us_pre;

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(PIN_INPUT_ESTOP_L, INPUT);
  pinMode(PIN_INPUT_ESTOP_R, INPUT);

  us_now = us_pre = millis();
}

void loop() {
  // 입력 처리
  swGreen.update();
  swRed.update();
  swStop.update();
  currentValue = analogRead(PIN_INPUT_CURRENT);

  if (swGreen.isDoubleClick()) {
    valueInput.sw_green = DOUBLE;
  } else if (swGreen.isLongClick()) {
    valueInput.sw_green = LONG;
    // PC 부팅 ON 처리
    // PC 부팅되었으면 동작되지 않도록 처리
  } else {
    valueInput.sw_green = RELEASED;
  }
  
  if (swRed.isDoubleClick()) {
    valueInput.sw_red = DOUBLE;
  } else if (swRed.isLongClick()) {
    valueInput.sw_red = LONG;
  } else {
    valueInput.sw_red = RELEASED;
  }
  
  if (swStop.isDoubleClick()) {
    valueInput.sw_stop = DOUBLE;
  } else if (swStop.isLongClick()) {
    valueInput.sw_stop = LONG;
  } else {
    valueInput.sw_stop = RELEASED;
  }
  
  if (digitalRead(PIN_INPUT_ESTOP_L)) {
    switch (stateEstopL) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_l = PUSHED;
        stateEstopL = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  } else {
    valueInput.estop_l = RELEASED;
    stateEstopL = STATE_ESTOP::ACTION;
  }
  
  if (digitalRead(PIN_INPUT_ESTOP_R)) {
    switch (stateEstopR) {
      case STATE_ESTOP::ACTION:
        valueInput.estop_r = PUSHED;
        stateEstopR = 1;
        break;
      case STATE_ESTOP::IDLE:
        break;
      default:
        break;
    }
  } else {
    valueInput.estop_r = RELEASED;
    stateEstopR = STATE_ESTOP::ACTION;
  }
  // 송신 데이터 처리
  if (valueInputBefore.estop_l != valueInput.estop_l ||
      valueInputBefore.estop_r != valueInput.estop_r ||
      valueInputBefore.sw_green != valueInput.sw_green ||
      valueInputBefore.sw_red != valueInput.sw_red ||
      valueInputBefore.sw_stop != valueInput.sw_stop) {
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

    Serial.write(buffer, SIZE_TOTAL_INPUT);
  }

  // 주기마다 아날로그 데이터 송신
  us_now = micros();
  #define SEND_RATE_US  1000000
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

    Serial.write(buffer, SIZE_TOTAL_INPUT);
  }
  // 수신 데이터 처리
  //   가. 송신 데이터에 대한 ACK
  //   나. 수신 데이터(출력)에 대한 현재 상태 요청에 대한 응답
  //   다. 수신 데이터(출력)에 대한 설정
  parseData();
  // 출력 처리
  ledGreen.Update();
  ledRed.Update();
  buzzer.Update();
  ledStart.Update();
  ledStop.Update();
  relBreak.Update();
}

bool parseData() {
  static int state = FSM_SERIAL::HEAD;
  static unsigned char packet[SIZE_TOTAL_OUTPUT] = {'\0', };

  switch (state) {
    case FSM_SERIAL::HEAD:
      if (Serial.available() >= SIZE_HEAD) {
        #if 1
        Serial.readBytes(packet+IDX_HEAD, SIZE_HEAD);
        #else
        packet[IDX_HEAD] = que.front();
        que.pop();
        #endif
        if (packet[IDX_HEAD] == DATA_HEAD) {
          state = FSM_SERIAL::TYPE;
        } else {
          state = FSM_SERIAL::HEAD;
        }
      }
      break;
    case FSM_SERIAL::TYPE:
      if (Serial.available() >= SIZE_TYPE) {
        #if 1
        Serial.readBytes(packet+IDX_TYPE, SIZE_TYPE);
        #else
        packet[IDX_TYPE] = que.front();
        que.pop();
        #endif
        
        state = FSM_SERIAL::TS;
      }
      break;
    case FSM_SERIAL::TS:
      if (Serial.available() >= SIZE_TS) {
        #if 1
        Serial.readBytes(packet+IDX_TS, SIZE_TS);
        #else
        for (int i=0; i<SIZE_TS; i++) {
          packet[IDX_TS+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_SERIAL::DATA;
      }
      break;
    case FSM_SERIAL::DATA:
      if (Serial.available() >= SIZE_DATA_OUTPUT) {
        #if 1
        Serial.readBytes(packet+IDX_DATA, SIZE_DATA_OUTPUT);
        #else
        for (int i=0; i<SIZE_DATA_OUTPUT; i++) {
          packet[IDX_DATA+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_SERIAL::CRC16;
      }
      break;
    case FSM_SERIAL::CRC16:
      if (Serial.available() >= SIZE_CRC16) {
        #if 1
        Serial.readBytes(packet+IDX_CRC16_OUTPUT, SIZE_CRC16);
        #else
        for (int i=0; i<SIZE_CRC16; i++) {
          packet[IDX_CRC16+i] = que.front();
          que.pop();
        }
        #endif

        state = FSM_SERIAL::TAIL;
      }
      break;
    case FSM_SERIAL::TAIL:
      if (Serial.available() >= SIZE_TAIL) {
        #if 1
        Serial.readBytes(packet+IDX_TAIL_OUTPUT, SIZE_TAIL);
        #else
        packet[IDX_TAIL] = que.front();
        que.pop();
        #endif
        if (packet[IDX_TAIL_OUTPUT] == DATA_TAIL) {
          state = FSM_SERIAL::OK;
        } else {
          state = FSM_SERIAL::HEAD;
        }
      }
      break;
    case FSM_SERIAL::OK:
      checksumData(packet);

      memset(packet, '\0', SIZE_TOTAL_OUTPUT);
      
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
  sscanf((const char*)(packet+IDX_CRC16_OUTPUT), "%04x", (unsigned int*)&crc16out);

  // 수신부 data의 crc16 계산
  unsigned short crc16 = CRC::CRC16((unsigned char*)(packet+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);

  if (crc16out == crc16) {
    memcpy(&valueOutput, packet+IDX_DATA, SIZE_DATA_OUTPUT);
    if (valueOutput.led_green.update) ledGreen.setOnOffTime(valueOutput.led_green.onTime, valueOutput.led_green.offTime, valueOutput.led_green.targetCount, valueOutput.led_green.lastState);
    if (valueOutput.led_red.update) ledRed.setOnOffTime(valueOutput.led_red.onTime, valueOutput.led_red.offTime, valueOutput.led_red.targetCount, valueOutput.led_red.lastState);
    if (valueOutput.buzzer.update) buzzer.setOnOffTime(valueOutput.buzzer.onTime, valueOutput.buzzer.offTime, valueOutput.buzzer.targetCount, valueOutput.buzzer.lastState);
    if (valueOutput.led_start.update) ledStart.setOnOffTime(valueOutput.led_start.onTime, valueOutput.led_start.offTime, valueOutput.led_start.targetCount, valueOutput.led_start.lastState);
    if (valueOutput.led_stop.update) ledStop.setOnOffTime(valueOutput.led_stop.onTime, valueOutput.led_stop.offTime, valueOutput.led_stop.targetCount, valueOutput.led_stop.lastState);
    if (valueOutput.rel_break.update) relBreak.setOnOffTime(valueOutput.rel_break.onTime, valueOutput.rel_break.offTime, valueOutput.rel_break.targetCount, valueOutput.rel_break.lastState);
  } else {
  }
}