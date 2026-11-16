#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "../common/Components.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "Collision.hpp"
#include "PhysicsMath.hpp"

struct ParticleCache {
    entt::entity entity;
    float x, y, theta;      // Position
    float vx, vy, omega;    // Velocity
    float vx0, vy0, omega0;
    float r;
    float invMass;
    float invInertia;
    float restitution;
    float staticFriction;
    float dynamicFriction;
    int type;        // 0=Circle, 1=Box
    float hw, hh;
};

struct GridCell {
    std::vector<int> particle_indices;
};

class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt);

private:
    const float CELL_SIZE = 30.0f;
    
    static const int HASH_SIZE = 4093; 
    std::vector<GridCell> hash_grid;
    std::vector<ParticleCache> cache;
    std::unordered_set<uint64_t> processed_pairs;

    int get_spatial_hash(int cx, int cy) const;
    void sync_from_registry(entt::registry& registry); 
    void refresh_grid_from_cache();
    void apply_changes_to_registry(entt::registry& registry);

    ForceRegistry force_registry;
    static void solve_sequential_impulse(ParticleCache& pA, ParticleCache& pB, const Collision::Manifold& result);
    static void resolve_contact(ParticleCache& pA, ParticleCache& pB);
    void solve_collisions();
    Collision::RaycastHit sweep_static_collisions(entt::registry& registry, 
                                   const Position& pos, 
                                   const Collider& collider, 
                                   float dx, float dy);
    void apply_continuous_movement(Position& pos, Velocity& vel, const RigidBody& body,
                                   const Collision::RaycastHit& hit, float dx, float dy, float dt);
};