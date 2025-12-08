#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"
#include "../common/Components.hpp"
#include "Components.hpp"
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
    cache.reserve(3000);
}

int PhysicsSystem::get_grid_index(float x, float y) const {
    int cx = (int)(x / CELL_SIZE);
    int cy = (int)(y / CELL_SIZE);
    if (cx < 0 || cx >= grid_width || cy < 0 || cy >= grid_height) return -1;
    return cx + cy * grid_width;
}

// --- PARTIE CORRIGÉE : BUILD GRID ---
void PhysicsSystem::build_grid(entt::registry& registry) {
    for(auto& cell : grid) cell.particle_indices.clear();
    cache.clear();
    
    auto view = registry.view<Position, Collider>();
    
    for(auto entity : view) {
        const auto& pos = view.get<Position>(entity);
        const auto& col = view.get<Collider>(entity);
        
        float vx = 0, vy = 0, ax = 0, ay = 0, invMass = 0, restitution = 1.0f;

        if (registry.all_of<RigidBody>(entity)) {
            const auto& body = registry.get<RigidBody>(entity);
            invMass = body.inverseMass;
            restitution = body.restitution;
        }
        if (registry.all_of<Velocity>(entity)) {
            const auto& vel = registry.get<Velocity>(entity);
            vx = vel.x; vy = vel.y;
        }
        if (registry.all_of<ForceAccumulator>(entity)) {
            const auto& force = registry.get<ForceAccumulator>(entity);
            if (invMass > 0) { ax = force.x * invMass; ay = force.y * invMass; }
        }

        int type = (col.type == ColliderType::CIRCLE) ? 0 : 1;
        
        // --- CORRECTION CRITIQUE ICI ---
        float hw = col.width * 0.5f;
        float hh = col.height * 0.5f;
        float r;

        if (type == 0) {
            r = col.radius; // Cercle
        } else {
            // Boite : Le "rayon" pour le cache doit couvrir toute la boite (Diagonale)
            // Sinon le test de distance échoue sur les bords du mur !
            r = std::sqrt(hw*hw + hh*hh); 
        }
        // -------------------------------

        cache.push_back({entity, pos.x, pos.y, vx, vy, ax, ay, invMass, restitution, type, r, hw, hh});
        int cache_idx = (int)cache.size() - 1;
        
        if (type == 0) { 
            int grid_idx = get_grid_index(pos.x, pos.y);
            if (grid_idx >= 0) grid[grid_idx].particle_indices.push_back(cache_idx);
        } 
        else {
            // Rasterisation du mur (ajout dans toutes les cases couvertes)
            int startX = (int)((pos.x - hw) / CELL_SIZE);
            int endX   = (int)((pos.x + hw) / CELL_SIZE);
            int startY = (int)((pos.y - hh) / CELL_SIZE);
            int endY   = (int)((pos.y + hh) / CELL_SIZE);

            startX = std::max(0, startX);
            endX   = std::min(grid_width - 1, endX);
            startY = std::max(0, startY);
            endY   = std::min(grid_height - 1, endY);

            for (int y = startY; y <= endY; ++y) {
                for (int x = startX; x <= endX; ++x) {
                    int grid_idx = x + y * grid_width;
                    grid[grid_idx].particle_indices.push_back(cache_idx);
                }
            }
        }
    }
}

void PhysicsSystem::apply_changes_to_registry(entt::registry& registry) {
    for(const auto& p : cache) {
        if (p.invMass > 0) {
            auto& pos = registry.get<Position>(p.entity);
            auto& vel = registry.get<Velocity>(p.entity);
            pos.x = p.x;
            pos.y = p.y;
            vel.x = p.vx;
            vel.y = p.vy;
        }
    }
}

void PhysicsSystem::update(entt::registry& registry, float dt) {
    const int sub_steps = 1; // Qualité physique
    float sub_dt = dt / sub_steps;

    registry.view<ForceAccumulator>().each([](auto& force) { force.x = 0; force.y = 0; });
    force_registry.apply_all(registry, sub_dt);

    build_grid(registry);

    for(int step = 0; step < sub_steps; ++step) {
        for(auto& p : cache) {
            if (p.invMass == 0) continue;
            p.vx += p.ax * sub_dt;
            p.vy += p.ay * sub_dt;
            p.x += p.vx * sub_dt;
            p.y += p.vy * sub_dt;
        }

        // Mise à jour partielle de la grille (uniquement les particules qui bougent)
        // Note: Pour les murs, ils restent en place, mais comme on clear les vecteurs,
        // idéalement il faudrait une grille statique séparée. 
        // MAIS pour faire simple et rapide ici : on rebuild juste les indices des particules.
        // Les murs ayant été ajoutés dans build_grid, si on clear tout ici, on les perd pour les steps suivants !
        //
        // CORRECTION DE BOUCLE : On ne doit PAS vider la grille entre les sub-steps si on ne re-add pas les murs.
        // Solution simple : On re-parcourt tout le cache (Murs inclus) pour remettre les indices.
        // C'est très rapide (itération linéaire sur vecteur).
        
        for(auto& cell : grid) cell.particle_indices.clear();
        
        for(int i = 0; i < (int)cache.size(); ++i) {
            const auto& p = cache[i];
            
            if (p.type == 0) { // Balle
                int grid_idx = get_grid_index(p.x, p.y);
                if (grid_idx >= 0) grid[grid_idx].particle_indices.push_back(i);
            } else { // Mur (re-rasterization rapide)
                int startX = (int)((p.x - p.hw) / CELL_SIZE);
                int endX   = (int)((p.x + p.hw) / CELL_SIZE);
                int startY = (int)((p.y - p.hh) / CELL_SIZE);
                int endY   = (int)((p.y + p.hh) / CELL_SIZE);
                
                // Clamp rapide
                if(startX < 0) startX=0; if(endX >= grid_width) endX=grid_width-1;
                if(startY < 0) startY=0; if(endY >= grid_height) endY=grid_height-1;

                for (int y = startY; y <= endY; ++y) {
                    for (int x = startX; x <= endX; ++x) {
                        grid[x + y * grid_width].particle_indices.push_back(i);
                    }
                }
            }
        }

        solve_collisions();
    }

    apply_changes_to_registry(registry);
}

void PhysicsSystem::solve_collisions() {
    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        
        if (pA.invMass == 0) continue; 

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
                        if (i == other_i) continue;

                        ParticleCache& pB = cache[other_i];

                        // Optimisation doublons (Seulement entre dynamiques)
                        if (pB.invMass > 0 && i > other_i) continue; 

                        // --- BROADPHASE AMELIORÉE (AABB) ---
                        // On utilise les dimensions réelles (hw/hh) au lieu de juste r
                        float extA_x = (pA.type == 0) ? pA.r : pA.hw;
                        float extB_x = (pB.type == 0) ? pB.r : pB.hw;
                        if (std::abs(pA.x - pB.x) > extA_x + extB_x) continue;

                        float extA_y = (pA.type == 0) ? pA.r : pA.hh;
                        float extB_y = (pB.type == 0) ? pB.r : pB.hh;
                        if (std::abs(pA.y - pB.y) > extA_y + extB_y) continue;

                        // Si cercle vs cercle, on garde l'optimisation au carré
                        if (pA.type == 0 && pB.type == 0) {
                            float rSum = pA.r + pB.r;
                            float dX = pA.x - pB.x;
                            float dY = pA.y - pB.y;
                            if (dX*dX + dY*dY > rSum*rSum) continue;
                        }

                        resolve_collision(pA, pB);
                    }
                }
            }
        }
    }
}

void PhysicsSystem::resolve_collision(ParticleCache& pA, ParticleCache& pB) {
    float nx = 0, ny = 0;
    float pen = 0;
    bool colliding = false;

    // CAS 1 : CERCLE (A) vs CERCLE (B)
    if (pA.type == 0 && pB.type == 0) {
        float dx = pA.x - pB.x;
        float dy = pA.y - pB.y;
        float distSq = dx*dx + dy*dy;
        float rSum = pA.r + pB.r;
        
        if (distSq < rSum * rSum && distSq > 0) {
            float dist = std::sqrt(distSq);
            nx = dx / dist;
            ny = dy / dist;
            pen = rSum - dist;
            colliding = true;
        }
    }
    // CAS 2 : CERCLE (A) vs BOITE (B)
    else if (pA.type == 0 && pB.type == 1) {
        // Limites de la boite
        float bMinX = pB.x - pB.hw;
        float bMaxX = pB.x + pB.hw;
        float bMinY = pB.y - pB.hh;
        float bMaxY = pB.y + pB.hh;

        // Point le plus proche sur la boite (Clamp)
        float closestX = std::max(bMinX, std::min(pA.x, bMaxX));
        float closestY = std::max(bMinY, std::min(pA.y, bMaxY));
        
        float dx = pA.x - closestX;
        float dy = pA.y - closestY;
        float distSq = dx*dx + dy*dy;
        
        // --- C'EST ICI QUE CA CHANGE ---
        
        // Si le centre du cercle est À L'INTÉRIEUR de la boite
        // (C'est le cas si le point le plus proche EST le centre de la balle)
        if (distSq < 0.0001f) {
            // On cherche le bord le plus proche pour éjecter
            float dLeft = std::abs(pA.x - bMinX);
            float dRight = std::abs(pA.x - bMaxX);
            float dTop = std::abs(pA.y - bMinY);
            float dBottom = std::abs(pA.y - bMaxY);

            // On trouve le minimum
            float minOverlap = dLeft;
            nx = -1.0f; ny = 0.0f; // Par défaut Gauche

            if (dRight < minOverlap) { minOverlap = dRight; nx = 1.0f; ny = 0.0f; }
            if (dTop < minOverlap)   { minOverlap = dTop;   nx = 0.0f; ny = -1.0f; }
            if (dBottom < minOverlap){ minOverlap = dBottom;nx = 0.0f; ny = 1.0f; }

            // Pénétration = Rayon + Distance pour sortir du mur
            pen = pA.r + minOverlap;
            colliding = true;
        } 
        else if (distSq < pA.r * pA.r) {
            // Collision standard sur le bord
            float dist = std::sqrt(distSq);
            pen = pA.r - dist;
            nx = dx / dist;
            ny = dy / dist;
            colliding = true;
        }
    }
    // CAS 3 : BOITE (A) vs CERCLE (B) (Même logique inversée)
    else if (pA.type == 1 && pB.type == 0) {
        float bMinX = pA.x - pA.hw;
        float bMaxX = pA.x + pA.hw;
        float bMinY = pA.y - pA.hh;
        float bMaxY = pA.y + pA.hh;

        float closestX = std::max(bMinX, std::min(pB.x, bMaxX));
        float closestY = std::max(bMinY, std::min(pB.y, bMaxY));
        
        float dx = pB.x - closestX;
        float dy = pB.y - closestY;
        float distSq = dx*dx + dy*dy;
        
        if (distSq < 0.0001f) {
            float dLeft = std::abs(pB.x - bMinX);
            float dRight = std::abs(pB.x - bMaxX);
            float dTop = std::abs(pB.y - bMinY);
            float dBottom = std::abs(pB.y - bMaxY);

            float minOverlap = dLeft;
            nx = -1.0f; ny = 0.0f; 

            if (dRight < minOverlap) { minOverlap = dRight; nx = 1.0f; ny = 0.0f; }
            if (dTop < minOverlap)   { minOverlap = dTop;   nx = 0.0f; ny = -1.0f; }
            if (dBottom < minOverlap){ minOverlap = dBottom;nx = 0.0f; ny = 1.0f; }

            // Attention : nx pointe vers l'extérieur de la boite A.
            // On veut repousser B, donc B doit aller dans le sens de la normale.
            // Dans apply_impulse, pB recule ( - imp * invMass).
            // Donc si nx = -1 (Gauche), pB va aller à Droite. C'est l'inverse qu'on veut.
            
            // Correction : La normale doit aller DE A VERS B.
            // Si le bord gauche est le plus proche, B est à gauche, donc on doit pousser vers la gauche (-1).
            // Mon apply_impulse fait : pB -= j * n.
            // Donc pour que B aille à gauche, n doit être positif (1).
            
            // On inverse la normale trouvée pour ce cas précis :
            nx = -nx; 
            ny = -ny;

            pen = pB.r + minOverlap;
            colliding = true;
        }
        else if (distSq < pB.r * pB.r) {
            float dist = std::sqrt(distSq);
            pen = pB.r - dist;
            nx = dx / dist; // Pointe de A vers B
            ny = dy / dist;
            colliding = true;
        }
    }

    if (colliding) {
        apply_impulse(pA, pB, nx, ny, pen);
    }
}

void PhysicsSystem::apply_impulse(ParticleCache& pA, ParticleCache& pB, float nx, float ny, float pen) {
    float totalMass = pA.invMass + pB.invMass;
    if (totalMass == 0) return;

    // Projection
    float moveX = (pen * nx) / totalMass;
    float moveY = (pen * ny) / totalMass;
    
    // Slop pour éviter la gigue : on ne corrige que si pénétration > 0.01
    if (pen > 0.01f) {
        pA.x += moveX * pA.invMass;
        pA.y += moveY * pA.invMass;
        pB.x -= moveX * pB.invMass;
        pB.y -= moveY * pB.invMass;
    }

    // Impulsion
    float rel_vx = pA.vx - pB.vx;
    float rel_vy = pA.vy - pB.vy;
    float velNormal = rel_vx * nx + rel_vy * ny;

    if (velNormal > 0) return;

    float e = std::max(pA.restitution, pB.restitution);
    float j = -(1.0f + e) * velNormal;
    j /= totalMass;

    float impX = j * nx;
    float impY = j * ny;

    pA.vx += impX * pA.invMass;
    pA.vy += impY * pA.invMass;
    pB.vx -= impX * pB.invMass;
    pB.vy -= impY * pB.invMass;
}

float PhysicsSystem::combine_restitution(float rA, float rB) { return rA*rB; }