#include "PinButton.h"

#define PIN_OUTPUT_SW_PC 22
#define PIN_OUTPUT_BUZZER 23 // wire(white)
#define PIN_OUTPUT_LED_GREEN 24 // wire(yellow)
#define PIN_OUTPUT_LED_RED 25 // wire(purple)

#define PIN_INPUT_ESTOP_L 30  // wire(black)
#define PIN_INPUT_ESTOP_R 31  // wire(green)
#define PIN_INPUT_SW_GREEN 33   // wire(gray)
#define PIN_INPUT_SW_RED 34   // wire(brown)

enum ConditionSwitch {
  RELEASED, PUSHED, DOUBLE, LONG
};

typedef struct _StateSwitch {
  int ESTOP_L;
  int ESTOP_R;
  int SW_GREEN;
  int SW_RED;
} StateSwitch;

StateSwitch stateSwitch;

PinButton btnSwGreen(PIN_INPUT_SW_GREEN, INPUT);
PinButton btnSwRed(PIN_INPUT_SW_RED, INPUT);

void setup() {
  Serial.begin(9600);
  pinMode(PIN_INPUT_ESTOP_L, INPUT);
  pinMode(PIN_INPUT_ESTOP_R, INPUT);
}

void loop() {
  btnSwGreen.update();
  btnSwRed.update();
  
  if (btnSwGreen.isDoubleClick()) {
    stateSwitch.SW_GREEN = DOUBLE;
  } else if (btnSwGreen.isLongClick()) {
    stateSwitch.SW_GREEN = LONG;
  } else {
    stateSwitch.SW_GREEN = RELEASED;
  }
  
  if (btnSwRed.isDoubleClick()) {
    stateSwitch.SW_RED = DOUBLE;
  } else if (btnSwRed.isLongClick()) {
    stateSwitch.SW_RED = LONG;
  } else {
    stateSwitch.SW_RED = RELEASED;
  }
  
  if (digitalRead(PIN_INPUT_ESTOP_L)) {
    stateSwitch.ESTOP_L = PUSHED;
  } else {
    stateSwitch.ESTOP_L = RELEASED;
  }
  
  if (digitalRead(PIN_INPUT_ESTOP_R)) {
    stateSwitch.ESTOP_R = PUSHED;
  } else {
    stateSwitch.ESTOP_R = RELEASED;
  }
}
