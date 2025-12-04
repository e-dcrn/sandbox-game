#pragma once

#include <string>

// ANSI escape codes for colors
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

void log_error(const std::string& msg, const char* error);
void log_info(const std::string& msg);
