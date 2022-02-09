const int inEStop = 30;
const int inLimitSW = 31;

int count = 0;

void setup() {
  pinMode(inEStop, INPUT);
  pinMode(inLimitSW, INPUT);

  Serial.begin(9600);
}

#define SW_NOT    0
#define SW_PUSHED 1

int isLimitSw = SW_NOT;
int isEStop = SW_NOT;

#define TX_BUF_SIZ  30

char bufTx[TX_BUF_SIZ];

void loop() {  
  if (HIGH == digitalRead(inEStop)) {
    isEStop = SW_PUSHED;
  } else {
    isEStop = SW_NOT;
  }
  
  if (HIGH == digitalRead(inLimitSW)) {
    isLimitSw = SW_PUSHED;
  } else {
    isLimitSw = SW_NOT;
  }

  sprintf(bufTx, "#%01d%01d#\n", isEStop, isLimitSw);

#define HEAD_IDX  0
#define TAIL_IDX  3
#define STX 0x02
#define ETX 0x03

  bufTx[HEAD_IDX] = STX;
  bufTx[TAIL_IDX] = ETX;

#if 0
  Serial.print(count++);
  Serial.print(": ");
#endif
  Serial.print(bufTx);

  delay(100);
}
