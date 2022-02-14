#include "PinButton.h"
#include "Flasher.h"

#include "protocol_serial.h"

#define PIN_OUTPUT_SW_PC 22
#define PIN_OUTPUT_BUZZER 23 // wire(white)
#define PIN_OUTPUT_LED_GREEN 24 // wire(yellow)
#define PIN_OUTPUT_LED_RED 25 // wire(purple)

#define PIN_INPUT_ESTOP_L 30  // wire(black)
#define PIN_INPUT_ESTOP_R 31  // wire(green)
#define PIN_INPUT_SW_GREEN 33   // wire(gray)
#define PIN_INPUT_SW_RED 34   // wire(brown)

ValueInput valueInput, valueInputBefore;
ValueOutput valueOutput, valueOutputBefore;

PinButton swGreen(PIN_INPUT_SW_GREEN, INPUT);
PinButton swRed(PIN_INPUT_SW_RED, INPUT);

Flasher buzzer(PIN_OUTPUT_BUZZER, 0, 0, INFINITE);
Flasher ledGreen(PIN_OUTPUT_LED_GREEN, 0, 0, INFINITE);
Flasher ledRed(PIN_OUTPUT_LED_RED, 0, 0, INFINITE);

int stateEstopL;
int stateEstopR;

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(PIN_INPUT_ESTOP_L, INPUT);
  pinMode(PIN_INPUT_ESTOP_R, INPUT);
}

void loop() {
  // 입력 처리
  swGreen.update();
  swRed.update();

  if (swGreen.isDoubleClick()) {
    valueInput.sw_green = DOUBLE;
  } else if (swGreen.isLongClick()) {
    valueInput.sw_green = LONG;
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
      valueInputBefore.sw_red != valueInput.sw_red) {
    valueInputBefore = valueInput;
    static char buffer[BUFSIZ];

    // 송신 포맷 생성
    buffer[IDX_HEAD] = DATA_HEAD;
    buffer[IDX_TYPE] = TYPE_CMD::CMD;
    *((unsigned long*)(buffer+IDX_TS)) = micros();
    sprintf(buffer+IDX_DATA, "%01d%01d%01d%01d", valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red);
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
    ledGreen.setOnOffTime(valueOutput.led_green.on, valueOutput.led_green.off, valueOutput.led_green.act);
    ledRed.setOnOffTime(valueOutput.led_red.on, valueOutput.led_red.off, valueOutput.led_red.act);
    buzzer.setOnOffTime(valueOutput.buzzer.on, valueOutput.buzzer.off, valueOutput.buzzer.act);
  } else {
  }
}