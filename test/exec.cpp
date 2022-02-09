#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <unistd.h>

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
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

    printf("$ roslaunch recipelab sim_pallete.launch 실행\n");
    ret = exec("roslaunch recipelab sim_pallete.launch");
    printf("ret: %s\n", ret.c_str());

    sleep(3);

    printf("$ rosnode kill --all\n");
    ret = exec("rosnode kill --all");
    printf("ret: %s\n", ret.c_str());

    printf("finished\n");

    return 0;
}