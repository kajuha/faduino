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

	int ledState;
	unsigned long previousMillis;

  public:
  Flasher(int outputPin, long onTime, long offTime, long targetCount, long lastState) {
    this->outputPin = outputPin;
    pinMode(outputPin, OUTPUT);     
      
    this->onTime = onTime;
    this->offTime = offTime;

    this->targetCount = targetCount;
    this->actualCount = 0;

    this->lastState = lastState;
    
    ledState = LOW; 
    previousMillis = 0;
  }

  void Update() {
    unsigned long currentMillis = millis();
    static int fsmLastState = OUTPUT_LASTSTATE::INIT;

    if (targetCount == INFINITE) {
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

      actualCount++;
    } else if ((ledState == LOW) && (currentMillis - previousMillis >= offTime)) {
      ledState = HIGH;
      previousMillis = currentMillis;
      digitalWrite(outputPin, ledState);
    }
  }

  void setOnOffTime(long onTime, long offTime, long targetCount, long lastState) {
    this->onTime = onTime;
    this->offTime = offTime;

    this->targetCount = targetCount;
    this->actualCount = 0;

    this->lastState = lastState;
    
    ledState = LOW; 
    previousMillis = 0;
  }
};
