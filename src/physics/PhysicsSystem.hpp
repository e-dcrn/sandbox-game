#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "../common/Components.hpp"
#include <unordered_map>
#include <vector>
#include "Collision.hpp"
#include "PhysicsMath.hpp"

struct ParticleCache {
    entt::entity entity;
    float x, y, theta;      // Position
    float vx, vy, omega;    // Velocity
    float r;         // Rayon (ou demi-taille)
    float invMass;
    float invInertia;
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
    void sync_from_registry(entt::registry& registry); 
    void refresh_grid_from_cache();
    void apply_changes_to_registry(entt::registry& registry);

    ForceRegistry force_registry;
    static Vect2 calculate_impulse_vector(const ParticleCache& pA, const ParticleCache& pB, 
        const Vect2& normal, const Vect2& contact_point);
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