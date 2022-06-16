#pragma once

#include "protocol_serial.h"

class Flasher {
  enum OUTPUT_LASTSTATE {
    INIT, SET, IDLE
  };

	int outputPin;
	long onTime;
	long offTime;
	long actualCount, targetCount;
  long lastState;
  int order;

	int ledState;
	unsigned long previousMillis;
  int fsmLastState;

  public:
    Flasher(int outputPin, long onTime, long offTime, long targetCount, long lastState, int order) {
    this->outputPin = outputPin;
    pinMode(outputPin, OUTPUT);

    this->onTime = onTime;
    this->offTime = offTime;

    this->targetCount = targetCount;
    this->actualCount = 0;

    this->lastState = lastState;

    fsmLastState = OUTPUT_LASTSTATE::INIT;
    previousMillis = 0;

#define ORDER_EN 0
    #if ORDER_EN
    // Flasher을 사용하지 않을 경우
    if (targetCount == STATE_ACT::DIRECT) {
      digitalWrite(outputPin, lastState);
      ledState = lastState;
    } else {
      if (order == FADUINO::ORDER::OFF_FIRST) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
    }
    #else
    ledState = LOW;
    #endif
  }

  void update() {
    unsigned long currentMillis = millis();

    if (targetCount == STATE_ACT::INFINITE) {
    } else if (targetCount == STATE_ACT::DIRECT) {
      #if ORDER_EN
      #else
      digitalWrite(outputPin, lastState);
      ledState = lastState;
      return;
      #endif
    } else if (targetCount <= actualCount) {
      switch (fsmLastState) {
        case OUTPUT_LASTSTATE::INIT:
          fsmLastState = OUTPUT_LASTSTATE::SET;
          break;
        case OUTPUT_LASTSTATE::SET:
          digitalWrite(outputPin, lastState);
          fsmLastState = OUTPUT_LASTSTATE::IDLE;
          break;
        case OUTPUT_LASTSTATE::IDLE:
          break;
        default:
          break;
      }
      return;
    } else {
    }

    fsmLastState = OUTPUT_LASTSTATE::INIT;

    if((ledState == HIGH) && (currentMillis - previousMillis >= onTime)) {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(outputPin, ledState);

      #if ORDER_EN
      if (order == FADUINO::ORDER::OFF_FIRST) {

      } else {
        actualCount++;
      }
      #else
      actualCount++;
      #endif
    } else if ((ledState == LOW) && (currentMillis - previousMillis >= offTime)) {
      ledState = HIGH;
      previousMillis = currentMillis;
      digitalWrite(outputPin, ledState);

      #if ORDER_EN
      if (order == FADUINO::ORDER::OFF_FIRST) {
        actualCount++;
      } else {

      }
      #else
      #endif
    } else {

    }
  }

  void setOnOffTime(long onTime, long offTime, long targetCount, long lastState, int order) {
    this->onTime = onTime;
    this->offTime = offTime;

    this->targetCount = targetCount;
    this->actualCount = 0;

    this->lastState = lastState;

    #if ORDER_EN
    if (order == FADUINO::ORDER::OFF_FIRST) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    #else
    ledState = LOW;
    #endif
    previousMillis = 0;
  }
};
