#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "Components.hpp"
#include "../common/Components.hpp"
#include <vector>
#include "Collision.hpp"

struct ParticleCache {
    entt::entity entity;
    float x, y;
    float r;
    float vx, vy; 
    float ax, ay; // <--- AJOUT : Accélération (Gravité)
    float invMass;
    float restitution;
};

struct GridCell {
    std::vector<int> particle_indices; 
};

class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt);

private:
    const float CELL_SIZE = 16.0f; 
    
    int grid_width;
    int grid_height;
    std::vector<GridCell> grid;
    std::vector<ParticleCache> cache;

    int get_grid_index(float x, float y) const;
    void build_grid(entt::registry& registry);

    ForceRegistry force_registry;

    static float combine_restitution(float rA, float rB);
    static void apply_impulse(ParticleCache& pA, ParticleCache& pB, float nx, float ny);
    static void resolve_collision(ParticleCache& pA, ParticleCache& pB);
    
    void handle_wall_collisions(ParticleCache& p);
    void solve_collisions();
    void apply_changes_to_registry(entt::registry& registry);
};