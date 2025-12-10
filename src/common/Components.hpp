#pragma once

#include <SDL.h>

struct Position {
    float x;
    float y;
};

struct Velocity {
    float x;
    float y;
};

struct ForceAccumulator {
    float x;
    float y;
};

struct RigidBody {
    float mass;
    float inverseMass;
    float restitution;
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