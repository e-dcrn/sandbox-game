#pragma once

#include <SDL.h>

struct Vect2 {
    float x;
    float y;
};

struct Position {
    float x;
    float y;
    float theta;
};

struct Velocity {
    float x;
    float y;
    float omega;
};

struct ForceAccumulator {
    float x;
    float y;
};

struct RigidBody {
    float mass;
    float inverseMass;
    float restitution;
    float inertia;
    float inverseInertia;
    float staticFriction;
    float dynamicFriction;
};

struct GravityAffected {
    float scale = 1.0f; 
};

enum class ColliderType {
    CIRCLE,
    BOX
};

struct Collider {
    ColliderType type;
    float radius;
    float width;
    float height;
};

struct AABB {
    float x, y;
    float w, h;
};

struct Circle {
    float radius;
    SDL_Color color;
};

struct Rectangle {
    float width;
    float height;
    SDL_Color color;
};

struct FollowsMouse {};

struct UseRaycast {};