#pragma once

enum class ScreenOutput {
    NO, DEFAULT, ALWAYS, TEMP, ERROR
};

void reprintf(ScreenOutput screenOutput, const char* format, ...);