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