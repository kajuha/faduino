#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <unistd.h>

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

int main() {
    std::string ret;

    // 쓰레드로 실행할 것
    printf("$ roslaunch recipelab sim_pallete.launch\n");
    ret = exec("roslaunch recipelab sim_pallete.launch");
    printf("ret: %s\n", ret.c_str());

    // sleep(3);

    // printf("$ rosnode kill --all\n");
    // ret = exec("rosnode kill --all");
    // printf("ret: %s\n", ret.c_str());

    // printf("finished\n");

    return 0;
}