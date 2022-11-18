#include <stdarg.h>
#include <stdio.h>
#include <sys/time.h>
#include <ctime>

#include "reprintf.h"

void reprintf(ScreenOutput screenOutput, const char* format, ...) {
    va_list argptr;

    struct timeval time_now{};
    gettimeofday(&time_now, nullptr);
    time_t ts_now;
    ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

    if (screenOutput == ScreenOutput::ALWAYS) {
#if 1
		printf("[%ld]", ts_now);
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::ERROR) {
#if 0
		printf("[%ld]", ts_now);
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::TEMP) {
#if 0
		printf("[%ld]", ts_now);
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::DEFAULT) {
#if 0
		printf("[%ld]", ts_now);
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::NO) {
    } else {
    }
}