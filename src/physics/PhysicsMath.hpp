#pragma once
#include "../common/Components.hpp"

inline Vect2 operator+(const Vect2& a, const Vect2& b) { return { a.x + b.x, a.y + b.y }; }
inline Vect2 operator-(const Vect2& a, const Vect2& b) { return { a.x - b.x, a.y - b.y }; }
inline Vect2 operator*(const Vect2& a, float s) { return { a.x * s, a.y * s }; }
inline Vect2 operator*(float s, const Vect2& a) { return { a.x * s, a.y * s }; }
inline Vect2 operator/(const Vect2& a, float s) { return { a.x / s, a.y / s }; }
inline Vect2 operator-(const Vect2& v) { return { -v.x, -v.y }; }

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

    inline Vect2 get_velocity_at_point(const Vect2& linear_vel, float angular_vel, const Vect2& r) {
        return { linear_vel.x - angular_vel * r.y, linear_vel.y + angular_vel * r.x };
    }

    inline float calculate_impulse_denominator(float invMass, float invInertia, const Vect2& r, const Vect2& normal) {
        float r_cross_n = cross(r, normal);
        return invMass + (r_cross_n * r_cross_n) * invInertia;
    }

    inline float calculate_total_impulse_denominator(
        float invMassA, float invInertiaA, const Vect2& rA,
        float invMassB, float invInertiaB, const Vect2& rB,
        const Vect2& normal) {
        return calculate_impulse_denominator(invMassA, invInertiaA, rA, normal) +
            calculate_impulse_denominator(invMassB, invInertiaB, rB, normal);
    }

    struct GridBounds { int startX, endX, startY, endY; };

    inline GridBounds compute_grid_bounds(float x, float y, float hw, float hh, float cellSize) {
        return {
            static_cast<int>(std::floor((x - hw) / cellSize)),
            static_cast<int>(std::floor((x + hw) / cellSize)),
            static_cast<int>(std::floor((y - hh) / cellSize)),
            static_cast<int>(std::floor((y + hh) / cellSize))
        };
    }

    inline void integrate_position(float& x, float& y, float& theta, float vx, float vy, float omega, float dt) {
        x += vx * dt;
        y += vy * dt;
        theta += omega * dt;
    }

    inline void apply_impulse_to_body(float& vx, float& vy, float& omega, 
                                  float invMass, float invInertia, 
                                  const Vect2& impulse, const Vect2& r) {
        vx += impulse.x * invMass;
        vy += impulse.y * invMass;
        omega += invInertia * cross(r, impulse);
    }

    inline void world_to_grid(float x, float y, float cellSize, int& outX, int& outY) {
        outX = static_cast<int>(std::floor(x / cellSize));
        outY = static_cast<int>(std::floor(y / cellSize));
    }

    inline Vect2 compute_aabb_half_dims(int type, float r, float hw, float hh, float angle) {
        if (type == 0) return { r, r };
        
        float cos_a = std::abs(std::cos(angle));
        float sin_a = std::abs(std::sin(angle));
        
        return {
            hw * cos_a + hh * sin_a,
            hw * sin_a + hh * cos_a 
        };
    }

}