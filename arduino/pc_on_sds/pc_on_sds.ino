#define LP_ON               1
#define LP_OFF              0

#define MILSEC_PER_SEC      1000
#define MILSEC_PER_MIN      60000

#define ERR_CODE_0          0 // PC not booted
#define ERR_CODE_1          1 //
#define ERR_CODE_2          2 //

const int LP_RELAY = 22; // PC SWITCH
const int LP_POW_STATUS_LED = 13;


static bool lp_status = LP_OFF;

void setup()
{
    Serial.begin(115200);
    pinMode(LP_RELAY,OUTPUT);
    pinMode(LP_POW_STATUS_LED, OUTPUT);
    digitalWrite(LP_RELAY, LOW);
    digitalWrite(LP_POW_STATUS_LED, LOW);
    Serial.println("[SYSTEM] Booting Pc...");
    bootLp();
}

void loop()
{
//  Serial.println("[SYSTEM] DUMMY LOOP");
  blink();
}

void bootLp(void)
{
    // Wait for steady LED
    delay(3*MILSEC_PER_SEC);

    if(!turnLpOn()){
        raiseError(ERR_CODE_0);
        digitalWrite(LP_POW_STATUS_LED, LOW);
    }else{
        Serial.println("[SYSTEM] PC ON.");
    }
}

bool turnLpOn(void)
{
    bool status = false;

    digitalWrite(LP_RELAY, HIGH);
    delay(3.5 * MILSEC_PER_SEC);
    digitalWrite(LP_RELAY, LOW);
    
    uint32_t st_time = millis();
    
    while(millis() - st_time <= 3 * MILSEC_PER_MIN)
    {
        if(chkLpStatus())
        {
            digitalWrite(LP_POW_STATUS_LED, LP_ON);            
            status = true;
            break;
        }
        else
        {
            delay(50);
        }
    }
    
    return status;
}

// check LP status (ON/OFF)
bool chkLpStatus(void){
    // // Serial Communication
    // String received = "";

    // if(Serial.available())
    // {
    //     // Send signal: chk 
    //     Serial.print("/system/boot_chk\n");
    //     delay(50);
    //     // Wait for response from LP
    //     received = Serial.readStringUntil('\n');

    //     if(received == "/system/ready") lp_status = true;
    // }

    // return lp_status;

    // DUMMY FUNCTION
    delay(2 * MILSEC_PER_SEC);
//    Serial.println("[SYSTEM] DUMMY_SYSTEM_CHK.");
    
    return true;
}

void raiseError(unsigned int error_code){
    switch(error_code)
    {
        case ERR_CODE_0:
            Serial.println("[SYSTEM] ERROR: PC BOOTING ERROR");
            break;
        case ERR_CODE_1:
            Serial.println("[SYSTEM] ERROR: ");
            break;
        case ERR_CODE_2:
            Serial.println("[SYSTEM] ERROR: ");
            break;
    }
}

// bool turnLpOff(void)
// {
//     // Send signal
// }

// bool chkAgvStatus(void)
// {
    
// }

void blink(void)
{
  digitalWrite(LP_POW_STATUS_LED, HIGH);
  delay(1000);
  digitalWrite(LP_POW_STATUS_LED, LOW);
  delay(1000);
}
