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

  bool isNew;
  bool usedDigitalWrite;

	int ledState;
	unsigned long previousMillis;
  unsigned long currentMillis;
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

    isNew = true;
    usedDigitalWrite = false;

#define ORDER_EN 1
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
    if (isNew) {
      previousMillis = currentMillis = millis();
      isNew = false;
    } else {
      currentMillis = millis();
    }

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

    if (ledState == HIGH) {
      if (currentMillis - previousMillis >= onTime) {
        previousMillis = currentMillis;
        usedDigitalWrite = false;
        #if ORDER_EN
        if (order == FADUINO::ORDER::ON_FIRST) {
          actualCount++;
        }
        #else
        actualCount++;
        #endif
      } else {
        if (!usedDigitalWrite) {
          usedDigitalWrite = true;
          ledState = LOW;
          digitalWrite(outputPin, ledState);
        }
      }
    } else if (ledState == LOW) {
      if (currentMillis - previousMillis >= offTime) {
        previousMillis = currentMillis;
        usedDigitalWrite = false;
        #if ORDER_EN
        if (order == FADUINO::ORDER::OFF_FIRST) {
          actualCount++;
        }
        #else

        #endif
      } else {
        if (!usedDigitalWrite) {
          usedDigitalWrite = true;
          ledState = HIGH;
          digitalWrite(outputPin, ledState);
        }
      }
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

    isNew = true;
    usedDigitalWrite = false;
  }
};
