#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "Components.hpp"
#include "../common/Components.hpp"


class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt, float time_scale);

private:
    ForceRegistry force_registry;
    void solve_collisions(entt::registry& registry);
    static float combine_restitution(float rA, float rB);
    static void resolve_pair(
        Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA,
        Position& posB, Velocity& velB, const RigidBody& bodyB, const Collider& colB
    );
};