#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"
#include <cmath>
#include <algorithm>

const int GRID_W = 1050;
const int GRID_H = 650;

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });

    grid_width = (int)(GRID_W / CELL_SIZE) + 1;
    grid_height = (int)(GRID_H / CELL_SIZE) + 1;
    
    grid.resize(grid_width * grid_height);
    for(auto& cell : grid) cell.particle_indices.reserve(16);
    
    cache.reserve(2500);
}

int PhysicsSystem::get_grid_index(float x, float y) const {
    int cx = (int)(x / CELL_SIZE);
    int cy = (int)(y / CELL_SIZE);
    if (cx < 0 || cx >= grid_width || cy < 0 || cy >= grid_height) return -1;
    return cx + cy * grid_width;
}

void PhysicsSystem::build_grid(entt::registry& registry) {
    for(auto& cell : grid) cell.particle_indices.clear();
    cache.clear();
    
    // AJOUT : On inclut ForceAccumulator dans la vue
    auto view = registry.view<Position, Velocity, RigidBody, Collider, ForceAccumulator>();
    
    for(auto entity : view) {
        const auto& pos = view.get<Position>(entity);
        const auto& vel = view.get<Velocity>(entity);
        const auto& body = view.get<RigidBody>(entity);
        const auto& col = view.get<Collider>(entity);
        const auto& force = view.get<ForceAccumulator>(entity); // <-- AJOUT
        
        float r = (col.type == ColliderType::CIRCLE) ? col.radius : col.width * 0.5f;

        // CALCUL DE L'ACCELERATION (F = ma -> a = F * invMass)
        float ax = force.x * body.inverseMass;
        float ay = force.y * body.inverseMass;

        // On stocke ax et ay dans le cache
        cache.push_back({entity, pos.x, pos.y, r, vel.x, vel.y, ax, ay, body.inverseMass, body.restitution});
        
        int idx = (int)cache.size() - 1;
        int grid_idx = get_grid_index(pos.x, pos.y);
        
        if (grid_idx >= 0) {
            grid[grid_idx].particle_indices.push_back(idx);
        }
    }
}

void PhysicsSystem::apply_changes_to_registry(entt::registry& registry) {
    for(const auto& p : cache) {
        auto& pos = registry.get<Position>(p.entity);
        auto& vel = registry.get<Velocity>(p.entity);
        pos.x = p.x;
        pos.y = p.y;
        vel.x = p.vx;
        vel.y = p.vy;
    }
}

void PhysicsSystem::update(entt::registry& registry, float dt) {
    const int sub_steps = 8; 
    float sub_dt = dt / sub_steps;

    // 1. Calculer les forces (Gravity)
    registry.view<ForceAccumulator>().each([](auto& force) { force.x = 0; force.y = 0; });
    force_registry.apply_all(registry, sub_dt);

    // 2. Construire le cache (Capture position, vélocité ET FORCES)
    build_grid(registry);

    // 3. Simulation sur Cache
    for(int step = 0; step < sub_steps; ++step) {
        
        for(auto& p : cache) {
            if (p.invMass == 0) continue;

            // --- CORRECTION CRITIQUE : APPLIQUER LA VITESSE ---
            p.vx += p.ax * sub_dt;
            p.vy += p.ay * sub_dt;

            p.x += p.vx * sub_dt;
            p.y += p.vy * sub_dt;

            handle_wall_collisions(p);
        }

        // Mise à jour de la grille (car les particules ont bougé)
        for(auto& cell : grid) cell.particle_indices.clear();
        for(int i = 0; i < (int)cache.size(); ++i) {
            int grid_idx = get_grid_index(cache[i].x, cache[i].y);
            if (grid_idx >= 0) grid[grid_idx].particle_indices.push_back(i);
        }

        solve_collisions();
    }

    apply_changes_to_registry(registry);
}

void PhysicsSystem::handle_wall_collisions(ParticleCache& p) {
    // Murs durs (Hardcoded pour la performance maximale)
    // Coordonnées basées sur tes murs Factory (50, 899, 125, etc.)

    // Sol (approx Y=550)
    if (p.y + p.r > 550.0f) {
        p.y = 550.0f - p.r;
        p.vy *= -0.5f;
    }
    // Plafond (approx Y=50)
    if (p.y - p.r < 50.0f) {
        p.y = 50.0f + p.r;
        p.vy *= -0.5f;
    }
    // Mur Gauche (approx X=125)
    if (p.x - p.r < 125.0f) {
        p.x = 125.0f + p.r;
        p.vx *= -0.5f;
    }
    // Mur Droite (approx X=899)
    if (p.x + p.r > 899.0f) {
        p.x = 899.0f - p.r;
        p.vx *= -0.5f;
    }
}

void PhysicsSystem::solve_collisions() {
    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        
        int idx = get_grid_index(pA.x, pA.y);
        if (idx == -1) continue;

        int cx = idx % grid_width;
        int cy = idx / grid_width;

        for(int dy = -1; dy <= 1; ++dy) {
            for(int dx = -1; dx <= 1; ++dx) {
                int nx = cx + dx;
                int ny = cy + dy;
                
                if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height) {
                    int neighbor_idx = nx + ny * grid_width;
                    const auto& cell = grid[neighbor_idx];

                    for(int other_i : cell.particle_indices) {
                        if (i >= other_i) continue;

                        ParticleCache& pB = cache[other_i];

                        float dist_sq = (pA.x - pB.x)*(pA.x - pB.x) + (pA.y - pB.y)*(pA.y - pB.y);
                        float rad_sum = pA.r + pB.r;
                        
                        if (dist_sq < rad_sum * rad_sum) {
                            resolve_collision(pA, pB);
                        }
                    }
                }
            }
        }
    }
}

void PhysicsSystem::apply_impulse(ParticleCache& pA, ParticleCache& pB, float nx, float ny) {
    float rel_vx = pA.vx - pB.vx;
    float rel_vy = pA.vy - pB.vy;
    
    float vel_normal = rel_vx * nx + rel_vy * ny;
    if (vel_normal > 0) return;

    float e = std::max(pA.restitution, pB.restitution);
    float j = -(1.0f + e) * vel_normal;
    j /= (pA.invMass + pB.invMass);

    float impulse_x = j * nx;
    float impulse_y = j * ny;

    pA.vx += impulse_x * pA.invMass;
    pA.vy += impulse_y * pA.invMass;
    pB.vx -= impulse_x * pB.invMass;
    pB.vy -= impulse_y * pB.invMass;
}

void PhysicsSystem::resolve_collision(ParticleCache& pA, ParticleCache& pB) {
    float dx = pA.x - pB.x;
    float dy = pA.y - pB.y;
    float dist = std::sqrt(dx*dx + dy*dy);
    
    if (dist == 0.0f) return;

    float nx = dx / dist;
    float ny = dy / dist;
    
    float pen = (pA.r + pB.r) - dist;
    
    float total_mass = pA.invMass + pB.invMass;
    if (total_mass > 0) {
        float move_x = (pen * nx) / total_mass;
        float move_y = (pen * ny) / total_mass;
        
        pA.x += move_x * pA.invMass;
        pA.y += move_y * pA.invMass;
        pB.x -= move_x * pB.invMass;
        pB.y -= move_y * pB.invMass;
    }

    apply_impulse(pA, pB, nx, ny);
}

float PhysicsSystem::combine_restitution(float rA, float rB) { return std::max(rA, rB); }