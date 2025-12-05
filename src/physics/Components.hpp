#pragma once

struct RigidBody {
    float mass;
    float inverseMass;
    float restitution;
};

struct ForceAccumulator {
    float x;
    float y;
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