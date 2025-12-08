#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"
#include "Components.hpp"
#include <vector>

// On enrichit le cache pour supporter toutes les formes
struct ParticleCache {
    entt::entity entity;
    
    // Position & Mouvement
    float x, y;
    float vx, vy;
    float ax, ay;
    
    // Propriétés Physiques
    float invMass;      // 0 = Mur (Statique), >0 = Balle (Dynamique)
    float restitution;  // Pour le rebond

    // Géométrie
    int type;           // 0 = CIRCLE, 1 = BOX
    float r;            // Rayon (si cercle)
    float hw, hh;       // Demi-largeur, Demi-hauteur (si boite)
};

struct GridCell {
    std::vector<int> particle_indices; 
};

class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt);

private:
    const float CELL_SIZE = 30.0f; // Un peu plus grand pour accommoder les murs
    
    int grid_width;
    int grid_height;
    std::vector<GridCell> grid;
    std::vector<ParticleCache> cache;

    int get_grid_index(float x, float y) const;
    void build_grid(entt::registry& registry);

    ForceRegistry force_registry;
    
    // Résolution générique
    void solve_collisions();
    
    // Fonctions bas niveau (statiques)
    static void resolve_collision(ParticleCache& pA, ParticleCache& pB);
    static void apply_impulse(ParticleCache& pA, ParticleCache& pB, float nx, float ny, float pen);
    
    // Plus de "handle_wall_collisions" hardcodé !
    
    void apply_changes_to_registry(entt::registry& registry);
    static float combine_restitution(float rA, float rB);
};