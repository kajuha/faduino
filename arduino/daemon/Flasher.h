#pragma once

#include "protocol_serial.h"

class Flasher {
	int ledPin;
	long OnTime;
	long OffTime;
	long CntActual, CntTarget;

	int ledState;
	unsigned long previousMillis;

  public:
  Flasher(int pin, long on, long off, long count) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);     
      
    OnTime = on;
    OffTime = off;

    CntTarget = count;
    CntActual = 0;
    
    ledState = LOW; 
    previousMillis = 0;
  }

  void Update() {
    unsigned long currentMillis = millis();

    if (CntTarget == CntActual) {
      return;
    } else if (CntTarget == INFINITE) {
    } else {
    }
     
    if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(ledPin, ledState);

      CntActual++;
    } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
      ledState = HIGH;
      previousMillis = currentMillis;
      digitalWrite(ledPin, ledState);
    }
  }

  void setOnOffTime(long on, long off, long act) {
    OnTime = on;
    OffTime = off;

    CntTarget = act;
    CntActual = 0;
    
    ledState = LOW; 
    previousMillis = 0;
  }
};
