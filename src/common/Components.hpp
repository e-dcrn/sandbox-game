#pragma once

#include <SDL.h>

struct Position {
    float x;
    float y;
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

struct RigidBody {
    float mass;
    float inverseMass;
    float restitution;
    float inertia;
    float invInertia;
};

struct Rotation {
    float angle;
    float omega;
};

struct ForceAccumulator {
    float x;
    float y;
    float torque;
};

struct Velocity {
    float x;
    float y;
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

struct FollowsMouse {};

struct UseRaycast {};