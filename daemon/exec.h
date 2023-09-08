#pragma once

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <boost/thread.hpp>

#define ROS_RUN "roslaunch recipelab faduinod.launch"
#define ROS_RUN_MINIMAL "roslaunch recipelab faduinod_minimal.launch"
#define ROS_CHECK "ps -ef|grep 'rosmaster'|grep -v 'grep'|awk '{print $2}'"
#define ROS_KILL "kill `ps -ef|grep 'rosmaster'|grep -v 'grep'|awk '{print $2}'`"
#define AMR_OFF "sudo shutdown -h now"

class Prog {
public:
    boost::thread *threadRos;

    static std::string execute(const char* cmd) {
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

    void rosRun() {
        threadRos = new boost::thread(execute, ROS_RUN);
    }

    void rosRunMinimal() {
        threadRos = new boost::thread(execute, ROS_RUN_MINIMAL);
    }
};