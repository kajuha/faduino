#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <boost/thread.hpp>

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    // https://pubs.opengroup.org/onlinepubs/009696799/functions/popen.html
    // popen() 함수는 호출하는 프로그램과 호출되는 프로그램사이에 파이프를 생성하고 데이터 읽기 쓰기를 할 수 있는 스트림을 생성하는 포인터를 리턴
    // mode가 "r"이면 프로세스가 실행될 때 스트림으로 들어오는 데이터를 읽을 수 있음
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

int main(int argc, char* argv[]) {
    std::string ret;

    printf("argv[1]: %s\n", argv[1]);

    #if 0
    const char* rosOpenCmd = "roslaunch recipelab sim_pallete.launch";
    #else
    const char* rosOpenCmd = "roslaunch recipelab pallete_can_web.launch";
    #endif
    const char* rosCheckCmd = "ps -ef|grep 'rosmaster'|grep -v 'grep'|awk '{print $2}'";
    const char* rosCloseCmd = "kill `ps -ef|grep 'rosmaster'|grep -v 'grep'|awk '{print $2}'`";
    
    printf("rosOpenCmd: %s\n", rosOpenCmd);
    printf("rosCheckCmd: %s\n", rosCheckCmd);
    printf("rosCloseCmd: %s\n", rosCloseCmd);

    boost::thread *threadRosOpen;

    while(true)
    {
        switch (getchar()) {
            case 'r':
                threadRosOpen = new boost::thread(exec, rosOpenCmd);
                printf("RosOpen\n");
                break;
            case 'c':
                ret = exec(rosCheckCmd);
                printf("RosCheck %s\n", ret.c_str());
                break;
            case 'k':
                ret = exec(rosCloseCmd);
                printf("RosClose\n");
                break;
            case 'q':
                printf("QUIT\n");
                goto QUIT;
            default:
            break;
        }
    }
    QUIT:

    printf("[c] threadRosOpen join\n");
    threadRosOpen->join();
    printf("[c] threadRosOpen joined\n");

    return 0;
}