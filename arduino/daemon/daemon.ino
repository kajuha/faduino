#include "PinButton.h"

#define PIN_OUTPUT_SW_PC 22
#define PIN_OUTPUT_BUZZER 23 // wire(white)
#define PIN_OUTPUT_LED_GREEN 24 // wire(yellow)
#define PIN_OUTPUT_LED_RED 25 // wire(purple)

#define PIN_INPUT_ESTOP_L 30  // wire(black)
#define PIN_INPUT_ESTOP_R 31  // wire(green)
#define PIN_INPUT_SW_GREEN 33   // wire(gray)
#define PIN_INPUT_SW_RED 34   // wire(brown)

PinButton btnEstopL(PIN_INPUT_ESTOP_L);
PinButton btnEstopR(PIN_INPUT_ESTOP_R);
PinButton btnSwGreen(PIN_INPUT_SW_GREEN);
PinButton btnSwRed(PIN_INPUT_SW_RED);

void setup() {
  Serial.begin(115200);
}

void loop() {
  btnEstopL.update();
  btnEstopR.update();
  btnSwGreen.update();
  btnSwRed.update();

  if (btnSwGreen.isClick()) {
    Serial.println("click");
  }
  if (btnSwGreen.isSingleClick()) {
    Serial.println("single");
  }
  if (btnSwGreen.isDoubleClick()) {
    Serial.println("double");
  }
  if (btnSwGreen.isLongClick()) {
    Serial.println("long");
  }
  if (btnSwGreen.isReleased()) {
    Serial.println("up");
  }
}
