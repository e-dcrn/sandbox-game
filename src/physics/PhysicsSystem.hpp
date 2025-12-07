#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "Components.hpp"
#include "../common/Components.hpp"
#include <unordered_map>


class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt, float time_scale);

private:
    const float CELL_SIZE = 6.0f; 
    std::unordered_map<long, std::vector<entt::entity>> spatial_grid;
    long get_hash_key(float x, float y) const;
    void build_grid(entt::registry& registry);

    ForceRegistry force_registry;
    static float combine_restitution(float rA, float rB);
    static void resolve_pair(
        Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA,
        Position& posB, Velocity& velB, const RigidBody& bodyB, const Collider& colB,
        float time_scale
    );
    void solve_collisions(entt::registry& registry, float time_scale);
};