#pragma once
#include "../common/Components.hpp"

namespace PhysicsMath {

    inline float dot(Vect2 a, Vect2 b) {
        return a.x * b.x + a.y * b.y;
    }
    
    inline Vect2 normalize(Vect2 v) {
        float len = std::sqrt(v.x * v.x + v.y * v.y);
        if (len == 0) return {0, 0};
        return {v.x / len, v.y / len};
    }
    
    inline Vect2 rotate(Vect2 v, float angle) {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return { v.x * c - v.y * s, v.x * s + v.y * c };
    }
    
    inline float cross(const Vect2& a, const Vect2& b) {
        return a.x * b.y - a.y * b.x;
    }
    
    inline Vect2 cross(float s, const Vect2& a) {
        return { -s * a.y, s * a.x };
    }
    
    inline float combine_restitution(float rA, float rB) {
        if (rA > 1.0f || rB > 1.0f) return std::max(rA, rB);
        return rA * rB;
    }

}