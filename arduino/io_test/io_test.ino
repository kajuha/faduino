#define PIN_ESTOP_L_IN 22
#define PIN_ESTOP_R_IN 23
#define PIN_SPARE1_IN 24
#define PIN_SPARE2_IN 25
#define PIN_START_IN 26
#define PIN_STOP_IN 27

#define PIN_PC_ON_OUT 29
#define PIN_BUZZER_OUT 37
#define PIN_START_LED_OUT 36
#define PIN_STOP_LED_OUT 35
#define PIN_BAT_RELAY_OUT 34
#define PIN_MD2_ESTOP_OUT 33
#define PIN_MD1_ESTOP_OUT 32
#define PIN_SPARE2_OUT 31
#define PIN_SPARE1_OUT 30

#define PIN_MD1_RS485_EN 6

int outPin = PIN_PC_ON_OUT;

int estop_l_in;
int estop_r_in;
int spare1_in;
int spare2_in;
int start_in;
int stop_in;

char tx_buf[BUFSIZ];

void setup() {
  pinMode(PIN_ESTOP_L_IN, INPUT);
  pinMode(PIN_ESTOP_R_IN, INPUT);
  pinMode(PIN_SPARE1_IN, INPUT);
  pinMode(PIN_SPARE2_IN, INPUT);
  pinMode(PIN_START_IN, INPUT);
  pinMode(PIN_STOP_IN, INPUT);

  pinMode(PIN_PC_ON_OUT, OUTPUT);
  pinMode(PIN_BUZZER_OUT, OUTPUT);
  pinMode(PIN_START_LED_OUT, OUTPUT);
  pinMode(PIN_STOP_LED_OUT, OUTPUT);
  pinMode(PIN_BAT_RELAY_OUT, OUTPUT);
  pinMode(PIN_MD2_ESTOP_OUT, OUTPUT);
  pinMode(PIN_MD1_ESTOP_OUT, OUTPUT);
  pinMode(PIN_SPARE2_OUT, OUTPUT);
  pinMode(PIN_SPARE1_OUT, OUTPUT);

  pinMode(PIN_MD1_RS485_EN, OUTPUT);
  digitalWrite(outPin, 0);

  Serial.begin(9600);
  Serial1.begin(9600);

  delay(1000);

  Serial.println("dbg started.");
}

void loop() {
  #if 1
  estop_l_in = digitalRead(PIN_ESTOP_L_IN);
  estop_r_in = digitalRead(PIN_ESTOP_R_IN);
  spare1_in = digitalRead(PIN_SPARE1_IN);
  spare2_in = digitalRead(PIN_SPARE2_IN);
  start_in = digitalRead(PIN_START_IN);
  stop_in = digitalRead(PIN_STOP_IN);

  sprintf(tx_buf, "%01d%01d%01d%01d%01d%01d\n", estop_l_in, estop_r_in, spare1_in, spare2_in, start_in, stop_in);
  Serial1.print(tx_buf);

  delay(100);
  #endif

  #if 0
  Serial1.println(outPin);
  digitalWrite(outPin, 1);
  delay(1000);
  digitalWrite(outPin, 0);
  delay(1000);
  #endif
}
