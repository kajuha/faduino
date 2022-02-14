#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "../../arduino/daemon/protocol_serial.h"

ValueInput valueInput;

int main() {
    char buffer[SIZE_TOTAL_INPUT];

    // 송신 포맷 생성
    buffer[IDX_HEAD] = DATA_HEAD;
    buffer[IDX_TYPE] = TYPE_CMD::CMD;
    *((int*)(buffer+IDX_TS)) = 0x12345678;
    sprintf(buffer+IDX_DATA, "%01d%01d%01d%01d", valueInput.estop_l, valueInput.estop_r, valueInput.sw_green, valueInput.sw_red);
    // crc16 계산
    unsigned short crc16in = CRC::CRC16((unsigned char*)(buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);
    printf("crc16in: %x\n", crc16in);
    sprintf(buffer+IDX_CRC16_INPUT, "%04x", crc16in);
    buffer[IDX_TAIL_INPUT] = DATA_TAIL;

    // 송신
    for (int i=0; i<sizeof(buffer); i++) {
        printf("[%02x]", buffer[i]);
    }
    printf("\n");

    // 수신
    for (int i=0; i<sizeof(buffer); i++) {
        printf("%c", buffer[i]);
    }
    printf("\n");

    // 가상으로 crc16 깨기
    // buffer[IDX_DATA] = 0;

    // 수신부 crc16 문자열 추출
    unsigned short crc16out;
    sscanf(buffer+IDX_CRC16_INPUT, "%04x", (unsigned int*)&crc16out);
    printf("crc16out: %x\n", crc16out);

    // 수신부 data의 crc16 계산
    unsigned short crc16 = CRC::CRC16((unsigned char*)(buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_INPUT);

    if (crc16out == crc16) {
        printf("crc16 matched.\n");
    } else {
        printf("crc16 not matched !!!\n");
    }

    // Output 문자열 crc16 체크
    #if 1
    char rx_buffer[] = {0x02, 0x01, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x06, 0x00, 0x31, 0x38, 0x37, 0x61, 0x03};
    
    unsigned short rx_crc16out;
    sscanf(rx_buffer+IDX_CRC16_OUTPUT, "%04x", (unsigned int*)&rx_crc16out);
    printf("rx_crc16out: %x\n", rx_crc16out);

    // 수신부 data의 crc16 계산
    unsigned short rx_crc16 = CRC::CRC16((unsigned char*)(rx_buffer+IDX_TYPE), SIZE_TYPE+SIZE_TS+SIZE_DATA_OUTPUT);

    if (rx_crc16out == rx_crc16) {
        printf("rx_crc16 matched.\n");
    } else {
        printf("rx_crc16 not matched !!!\n");
    }
    #endif

    return 0;
}