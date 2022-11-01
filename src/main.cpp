#include <ros/ros.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#include <boost/thread.hpp>
#include <queue>

#include "faduino/PanelOut.h"

#include "faduino/main.h"

#include "../arduino/daemon/include/protocol_serial.h"

std::queue<ValueOutput> queValueOutput;

bool servicePanelOutCallback(faduino::PanelOut::Request &req, faduino::PanelOut::Response &res) {
    static ValueOutput srvValueOutput;
    #if 0
    printf("[req:buzzer   ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.buzzer.onTime, req.buzzer.offTime, req.buzzer.targetCount, req.buzzer.lastState, req.buzzer.update, req.buzzer.order);
    printf("[req:md_power ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.md_power.onTime, req.md_power.offTime, req.md_power.targetCount, req.md_power.lastState, req.md_power.update, req.md_power.order);
    printf("[req:md_estop ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.md_estop.onTime, req.md_estop.offTime, req.md_estop.targetCount, req.md_estop.lastState, req.md_estop.update, req.md_estop.order);
    printf("[req:led_start] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.led_start.onTime, req.led_start.offTime, req.led_start.targetCount, req.led_start.lastState, req.led_start.update, req.led_start.order);
    printf("[req:led_stop ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.led_stop.onTime, req.led_stop.offTime, req.led_stop.targetCount, req.led_stop.lastState, req.led_stop.update, req.led_stop.order);
    printf("[req:bat_relay] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        req.bat_relay.onTime, req.bat_relay.offTime, req.bat_relay.targetCount, req.bat_relay.lastState, req.bat_relay.update, req.bat_relay.order);
    #endif

    #if 0
    printf("sizeof(srvValueOutput): %ld\n", sizeof(srvValueOutput));
    printf("sizeof(req): %ld\n", sizeof(req));
    #endif

    memcpy(&srvValueOutput, &req, sizeof(srvValueOutput));
    queValueOutput.push(srvValueOutput);

    #if 1
    printf("[srv:buzzer   ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.buzzer.onTime, srvValueOutput.buzzer.offTime, srvValueOutput.buzzer.targetCount, srvValueOutput.buzzer.lastState, srvValueOutput.buzzer.update, srvValueOutput.buzzer.order);
    printf("[srv:md_power ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.md_power.onTime, srvValueOutput.md_power.offTime, srvValueOutput.md_power.targetCount, srvValueOutput.md_power.lastState, srvValueOutput.md_power.update, srvValueOutput.md_power.order);
    printf("[srv:md_estop ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.md_estop.onTime, srvValueOutput.md_estop.offTime, srvValueOutput.md_estop.targetCount, srvValueOutput.md_estop.lastState, srvValueOutput.md_estop.update, srvValueOutput.md_estop.order);
    printf("[srv:led_start] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.led_start.onTime, srvValueOutput.led_start.offTime, srvValueOutput.led_start.targetCount, srvValueOutput.led_start.lastState, srvValueOutput.led_start.update, srvValueOutput.led_start.order);
    printf("[srv:led_stop ] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.led_stop.onTime, srvValueOutput.led_stop.offTime, srvValueOutput.led_stop.targetCount, srvValueOutput.led_stop.lastState, srvValueOutput.led_stop.update, srvValueOutput.led_stop.order);
    printf("[srv:bat_relay] on: %5d, off: %5d, cnt: %2d, last: %2d, update: %2d, order: %2d\n",
        srvValueOutput.bat_relay.onTime, srvValueOutput.bat_relay.offTime, srvValueOutput.bat_relay.targetCount, srvValueOutput.bat_relay.lastState, srvValueOutput.bat_relay.update, srvValueOutput.bat_relay.order);
    #endif
    
#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

int server_sock;

// 시그널 핸들러(클라이언트 TCP 해제)
#include <signal.h>
int readWriteInfinite = 0;
int serverOpen = 1;
int isConnectedServer = 0;

void sigpipe_handler(int sig) {
    // signal(SIGPIPE, sigpipe_handler);
    printf("[c] received SIGPIPE: %d \n", sig);
    readWriteInfinite = 0;
    isConnectedServer = 0;
}

void sigint_handler(int sig) {
    // signal(SIGINT, sigint_handler);
    printf("[c] received SIGINT: %d \n", sig);
    close(server_sock);
    serverOpen = 0;
    isConnectedServer = 0;
}

void fThread(std::string host_name, int tcp_port) {
    // TCP 통신 관련 변수
    struct sockaddr_in server_addr;
    int ret;
    int recvLen;

    printf("[c] serverOpen while start (%d line)\n", __LINE__);
    while (serverOpen) {
        printf("[c] serverOpen while started (%d line)\n", __LINE__);

        // 소켓 열기
        if ((server_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            // perror("[c] socket open error, ");
            printf("[c] socket open error\n");
            // return;
            continue;
        }
        printf("[c] socket open\n");

        // time_wait 제거하기
        int option = 1;
        setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        // 시그널 핸들러 등록
        // signal(SIGINT, sigint_handler);

        // 시그널 핸들러 등록
        readWriteInfinite = 1;
        signal(SIGPIPE, sigpipe_handler);
        
        #if 0
        struct hostent *host;
        host = gethostbyname(host_name.c_str());
        if (host == NULL) {
            perror(host_name.c_str());
            abort();
        }
        server_addr.sin_addr.s_addr = *(long *)(host->h_addr_list[0]);
        printf("domain: %s == ip: %s\n", host_name.c_str(), inet_ntoa(server_addr.sin_addr));
        #else
        server_addr.sin_addr.s_addr = inet_addr(host_name.c_str());
        printf("ip: %s, port: %d\n", host_name.c_str(), tcp_port);
        #endif
        
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(tcp_port);

        printf("[c] server_socket : %d\n", server_sock);
        // 연결 수락
        if (connect(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            // perror("[c] connect error, ");
            printf("[c] connect error\n");
            // goto PROGRAM_END;
            // return;
            // printf( "[c] server_sock: %d\n", server_sock);
            close(server_sock);
            usleep(500000);
            continue;
        } else {
            printf("[c] server ip : %s\n", inet_ntoa(server_addr.sin_addr));
            printf("[c] server connect\n");
            isConnectedServer = 1;
        }

        struct timeval time_now{};
        gettimeofday(&time_now, nullptr);
        time_t ts_now;
        ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

        uint8_t recvBuffer[BUFSIZ] = {'\0', };
        queValueOutput = std::queue<ValueOutput>();

        printf("[c] readWriteInfinite while start (%d line)\n", __LINE__);
        while (readWriteInfinite) {
            // printf("[c] readWriteInfinite while started (%d line)\n", __LINE__);

            if (queValueOutput.size()) {
                printf("queValueOutput size: %ld\n", queValueOutput.size());
                ValueOutput valueOutput = queValueOutput.front();
                queValueOutput.pop();

                static uint8_t buffer[SIZE_TOTAL_OUTPUT];

                buffer[IDX_HEAD] = DATA_HEAD;
                uint8_t type = TYPE_CMD::RELAY;
                memcpy(buffer+IDX_TYPE, &type, SIZE_TYPE);
                uint32_t timestamp = 0;
                memcpy(buffer+IDX_TS, &timestamp, SIZE_TS);
                memcpy(buffer+IDX_DATA, &valueOutput, SIZE_DATA_OUTPUT);
                uint32_t crc16 = 0x55AA55AA;
                memcpy(buffer+IDX_CRC16_OUTPUT, &crc16, SIZE_CRC16);
                buffer[IDX_TAIL_OUTPUT] = DATA_TAIL;

                send(server_sock, buffer, SIZE_TOTAL_OUTPUT, 0);
            }

            recvLen = recv(server_sock, recvBuffer, BUFSIZ, MSG_PEEK|MSG_DONTWAIT);
            if (recvLen > 0) {
                // 버퍼개수 읽기(MSG_PEEK는 버퍼확인용)
                memset(recvBuffer, '\0', sizeof(recvBuffer));
                recvLen = recv(server_sock, recvBuffer, BUFSIZ, 0);
                #if 1
                printf("recvLen: %d ", recvLen);
                for (int i=0; i<recvLen; i++) {
                    printf("[%02x]", recvBuffer[i]);
                }
                printf("\n");
                #endif
            } else if (recvLen == 0) {
                // 참고자료
                // recv를 호출했을때 0이 리턴되거나 send를 호출했을때 -1로 에러리턴이면 상대편 연결이 종료된 것

                // 커넥션중일 때는 MSG_PEEK|MSG_DONTWAIT에서 아무것도 송신안할 경우 -1이 발생하였음
                // 커넥션을 강제해제했을 경우 0이 발생하였음
                // MSG_DONTWIT를 추가하지 않았을 경우에는 무한대기가 되었음
                // 우선 이케이스에는 0을 커넥션 종료로 처리함
                #if 0
                printf("[c] server disconnect error, recv: %d \n", recvLen);
                readWriteInfinite = 0;
                isConnectedServer = 0;
                continue;
                #else
                readWriteInfinite = 0;
                serverOpen = 1;
                isConnectedServer = 0;
                break;
                #endif
            } else {
            }
        }
        printf("[c] tcp read/write end\n");

        ret = close(server_sock);
        printf("[c] socket closed, ret: %d\n", ret);
        // 소켓을 정상적으로 너무 빨리 닫고 재 열기할 경우
        // 해당 포트가 이미 사용중이라는 표시가 나타날 수 있음
#define RECONNECT_US 500000
        usleep(RECONNECT_US);
    }
}

int main(int argc, char* argv[]) {
    std::string nodeName = "faduino";
    ros::init(argc, argv, nodeName.c_str());
    ros::NodeHandle nh("~");
    
    int main_hz;
    std::string host_name;
    int tcp_port;

    // 파라미터 초기화
    #if 1
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~host_name", host_name);
    ros::param::get("~tcp_port", tcp_port);
    #else
    nh.getParam("main_hz", main_hz);
    nh.getParam("host_name", host_name);
    nh.getParam("tcp_port", tcp_port);
    #endif

    boost::thread threadTcp(fThread, host_name, tcp_port);

    // 패널 출력 서비스
    ros::ServiceServer service_panel_out = nh.advertiseService("/faduino/panel_out", servicePanelOutCallback);
    
    ros::Rate r(main_hz);

    double ts_now, ts_pre;

    ts_now = ts_pre = ros::Time::now().toSec();

    while (ros::ok()) {
        ts_now = ros::Time::now().toSec();

#define TEST_LOOP_SEC 1.0 
        if ((ts_now-ts_pre) > TEST_LOOP_SEC) {
            ts_pre = ts_now;
        }

        ros::spinOnce();
        r.sleep();
    }

    printf("[c] threadTcp join\n");
    close(server_sock);
    readWriteInfinite = 0;
    serverOpen = 0;
    isConnectedServer = 0;
    threadTcp.join();
    printf("[c] threadTcp joined\n");

    return 0;
}