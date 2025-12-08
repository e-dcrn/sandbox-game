#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "Components.hpp"
#include "../common/Components.hpp"
#include <unordered_map>
#include <vector>
#include "Collision.hpp"

struct ParticleCache {
    entt::entity entity;
    float x, y;      // Position
    float vx, vy;    // Velocity
    float r;         // Rayon (ou demi-taille)
    float invMass;
    float restitution;
    int type;        // 0=Circle, 1=Box
    float hw, hh;    // Pour les boites
};

struct GridCell {
    std::vector<int> particle_indices; // Stocke l'index dans le vecteur 'cache'
};

class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt);

private:
    const float CELL_SIZE = 30.0f; // Cellule plus large pour moins de checks
    
    // TABLE DE HACHAGE FIXE (Supporte monde infini par enroulement)
    static const int HASH_SIZE = 4093; 
    std::vector<GridCell> hash_grid;
    std::vector<ParticleCache> cache; // Toutes les données contiguës ici

    int get_spatial_hash(int cx, int cy) const;
    void build_grid_and_cache(entt::registry& registry);
    void apply_changes_to_registry(entt::registry& registry);

    ForceRegistry force_registry;
    static float combine_restitution(float rA, float rB);
    static void apply_impulse_response(ParticleCache& pA, ParticleCache& pB, const Vect2& normal);
    static void resolve_pair(ParticleCache& pA, ParticleCache& pB);
    void solve_collisions();
    RayHit sweep_static_collisions(entt::registry& registry, 
                                   const Position& pos, 
                                   const Collider& collider, 
                                   float dx, float dy);
    void apply_continuous_movement(Position& pos, Velocity& vel, const RigidBody& body,
                                   const RayHit& hit, float dx, float dy, float dt);
};