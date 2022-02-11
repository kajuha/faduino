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

Flasher buzzer(PIN_OUTPUT_BUZZER, 0, 0);
Flasher ledGreen(PIN_OUTPUT_LED_GREEN, 0, 0);
Flasher ledRed(PIN_OUTPUT_LED_RED, 0, 0);

int stateEstopL;
int stateEstopR;

bool parseData();

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
      *((unsigned long*)(buffer+IDX_TS)) = millis();
      sprintf(buffer+IDX_DATA, "%01d%01d%01d%01d", valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red);
      // crc16 계산
      unsigned short crc16in = CRC::CRC16((unsigned char*)(buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);
      sprintf(buffer+IDX_CRC16, "%04x", crc16in);
      buffer[IDX_TAIL] = DATA_TAIL;

      Serial.write(buffer, SIZE_TOTAL_INPUT);
    }
  // 수신 데이터 처리
  parseData();
  // 출력 처리
  ledGreen.Update();
  ledRed.Update();
  buzzer.Update();
}

bool parseData() {
  static int state = FSM_SERIAL::HEAD;
  static unsigned char packet[SIZE_TOTAL_INPUT] = {'\0', };

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
      if (Serial.available() >= SIZE_DATA_INPUT) {
        #if 1
        Serial.readBytes(packet+IDX_DATA, SIZE_DATA_INPUT);
        #else
        for (int i=0; i<SIZE_DATA_INPUT; i++) {
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
        Serial.readBytes(packet+IDX_CRC16, SIZE_CRC16);
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
        Serial.readBytes(packet+IDX_TAIL, SIZE_TAIL);
        #else
        packet[IDX_TAIL] = que.front();
        que.pop();
        #endif
        if (packet[IDX_TAIL] == DATA_TAIL) {
          state = FSM_SERIAL::OK;
        } else {
          state = FSM_SERIAL::HEAD;
        }
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