#include "Logger.hpp"
#include <iostream>

void log_error(const std::string& msg, const char* error) {
    std::cerr << RED << "[ERROR] " << msg << ": " << error << RESET << std::endl;
}

void log_info(const std::string& msg) {
    std::cout << GREEN << "[INFO] " << msg << RESET << std::endl;
}
