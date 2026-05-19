#pragma once
#include <iostream>
#include <cmath>
#include <string>
#include "../physics/Collision.hpp"

#define GREEN "\033[32m"
#define RED "\033[31m"
#define RESET "\033[0m"

const float EPSILON = 0.005f;

inline bool IsClose(float a, float b) {
    return std::abs(a - b) < EPSILON;
}

template <typename T>
bool IsVecClose(const T& a, const T& b) {
    return a == b; 
}

template <>
bool IsVecClose<Vect2>(const Vect2& a, const Vect2& b) {
    return IsClose(a.x, b.x) && IsClose(a.y, b.y);
}

inline void ExpectTrue(bool condition, const std::string& testName) {
    if (condition) {
        std::cout << GREEN << "[PASS] " << RESET << testName << std::endl;
    } else {
        std::cout << RED << "[FAIL] " << RESET << testName << std::endl;
    }
}