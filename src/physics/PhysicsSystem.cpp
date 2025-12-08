#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });

    hash_grid.resize(HASH_SIZE);
    for(auto& cell : hash_grid) cell.particle_indices.reserve(16);
    cache.reserve(3000);
}

void PhysicsSystem::update(entt::registry& registry, float dt) {

    const int sub_steps = 1;
    float sub_dt = dt / sub_steps;

    for(int i = 0; i < sub_steps; ++i){
        registry.view<ForceAccumulator>().each([](auto& force) {
            force.x = 0; force.y = 0;
        });

        force_registry.apply_all(registry, sub_dt);

        auto view = registry.view<RigidBody, Position, Velocity, Collider, ForceAccumulator>();
        view.each([&registry, sub_dt, this](auto entity, const auto& body, auto& pos, auto& vel, const auto& col, const auto& forces) {
            if (body.inverseMass == 0) return;

            float acc_x = forces.x * body.inverseMass;
            float acc_y = forces.y * body.inverseMass;
            vel.x += acc_x * sub_dt;
            vel.y += acc_y * sub_dt;
            float dx = vel.x * sub_dt;
            float dy = vel.y * sub_dt;

            // RayHit wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            RayHit wall_hit;
            wall_hit.hit = false;
            if (registry.any_of<UseRaycast>(entity)) {
                wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            }
            apply_continuous_movement(pos, vel, body, wall_hit, dx, dy, sub_dt);
        });

        build_grid_and_cache(registry);
        solve_collisions();
        apply_changes_to_registry(registry);
    }
}

int PhysicsSystem::get_spatial_hash(int cx, int cy) const {
    const int p1 = 73856093;
    const int p2 = 19349663;
    int n = ((cx * p1) ^ (cy * p2)) % HASH_SIZE;
    if (n < 0) n += HASH_SIZE;
    return n;
}

void PhysicsSystem::build_grid_and_cache(entt::registry& registry) {
    // 1. Reset (Rapide)
    for(auto& cell : hash_grid) cell.particle_indices.clear();
    cache.clear();

    // 2. Copie Registry -> Cache
    // On prend TOUT (Murs et Balles) pour que les collisions soient gérées dans le cache
    auto view = registry.view<Position, Collider>();
    
    view.each([this, &registry](auto entity, const auto& pos, const auto& col) {
        
        float vx=0, vy=0, invMass=0, rest=1.0f;
        
        if (registry.all_of<Velocity>(entity)) {
            const auto& vel = registry.get<Velocity>(entity);
            vx = vel.x; vy = vel.y;
        }
        if (registry.all_of<RigidBody>(entity)) {
            const auto& body = registry.get<RigidBody>(entity);
            invMass = body.inverseMass;
            rest = body.restitution;
        }

        int type = (col.type == ColliderType::CIRCLE) ? 0 : 1;
        float r = col.radius;
        float hw = col.width * 0.5f;
        float hh = col.height * 0.5f;
        
        // Calcul du rayon englobant pour la grille (Broadphase)
        float cache_r = (type == 0) ? r : std::sqrt(hw*hw + hh*hh);

        cache.push_back({entity, pos.x, pos.y, vx, vy, cache_r, invMass, rest, type, hw, hh});
        int idx = (int)cache.size() - 1;
        
        float obj_hw = (type == 0) ? r : hw;
        float obj_hh = (type == 0) ? r : hh;

        // On convertit les coordonnées Monde -> Coordonnées Grille
        // std::floor est important pour gérer correctement les coordonnées négatives (hors écran)
        int min_grid_x = (int)std::floor((pos.x - obj_hw) / CELL_SIZE);
        int max_grid_x = (int)std::floor((pos.x + obj_hw) / CELL_SIZE);
        int min_grid_y = (int)std::floor((pos.y - obj_hh) / CELL_SIZE);
        int max_grid_y = (int)std::floor((pos.y + obj_hh) / CELL_SIZE);

        // On insère l'objet dans TOUTES les cellules qu'il touche
        for (int y = min_grid_y; y <= max_grid_y; ++y) {
            for (int x = min_grid_x; x <= max_grid_x; ++x) {
                int hash = get_spatial_hash(x, y);
                hash_grid[hash].particle_indices.push_back(idx);
            }
        }
    });
}

void PhysicsSystem::apply_changes_to_registry(entt::registry& registry) {
    // Copie Cache -> Registry (Seulement pour les dynamiques)
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

float PhysicsSystem::combine_restitution(float rA, float rB) {
    if (rA > 1.0f || rB > 1.0f) return std::max(rA, rB);
    return rA * rB;
}

// Cette fonction ne touche PAS aux positions (pos.x), seulement aux vitesses (vel.x)
void PhysicsSystem::apply_impulse_response(ParticleCache& pA, ParticleCache& pB, const Vect2& normal) {
    float relVelX = pA.vx - pB.vx;
    float relVelY = pA.vy - pB.vy;
    float velAlongNormal = relVelX * normal.x + relVelY * normal.y;

    if (velAlongNormal > 0) return;

    float e = combine_restitution(pA.restitution, pB.restitution);
    if (velAlongNormal > -0.1f && velAlongNormal < 0.1f) e = 0.0f;

    float j = -(1.0f + e) * velAlongNormal;
    float totalMass = pA.invMass + pB.invMass;
    if (totalMass > 0) j /= totalMass;

    float impX = normal.x * j;
    float impY = normal.y * j;

    pA.vx += impX * pA.invMass;
    pA.vy += impY * pA.invMass;
    pB.vx -= impX * pB.invMass;
    pB.vy -= impY * pB.invMass;
}

void PhysicsSystem::resolve_pair(ParticleCache& pA, ParticleCache& pB) {
    CollisionResult result = {false, {0,0}, 0};

    // --- 1. DÉTECTION (Adaptation des données Cache vers tes fonctions de check) ---
    // Note : p.hw et p.hh sont les demi-largeurs, tes fonctions attendent souvent width/height (x2)
    
    if (pA.type == 0 && pB.type == 0) { // Circle vs Circle
        result = check_circle_circle({pA.x, pA.y}, pA.r, {pB.x, pB.y}, pB.r);
    }
    else if (pA.type == 0 && pB.type == 1) { // Circle vs Box
        result = check_circle_box({pA.x, pA.y}, pA.r, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f});
    }
    else if (pA.type == 1 && pB.type == 0) { // Box vs Circle
        result = check_circle_box({pB.x, pB.y}, pB.r, {pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f});
        // Inversion normale car A vs B
        result.normal_vector.x *= -1.0f;
        result.normal_vector.y *= -1.0f;
    }
    else { // Box vs Box
        result = check_box_box({pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f});
    }

    // --- 2. RÉSOLUTION (Ton Algorithme Exact) ---

    if (result.is_colliding) {
        float invMassA = pA.invMass;
        float invMassB = pB.invMass;
        float totalMass = invMassA + invMassB;

        if (totalMass == 0.0f) return;

        float relVelX = (pA.vx - pB.vx);
        float relVelY = (pA.vy - pB.vy);
        float velAlongNormal = relVelX * result.normal_vector.x + relVelY * result.normal_vector.y;
        
        if (velAlongNormal > 0) return;

        bool solved = false;
        // Calcul du "Temps à rembobiner" pour trouver le moment exact de l'impact
        float timeToRewind = result.depth / -velAlongNormal;
        const float EPSILON = 0.001f;
        
        if (velAlongNormal < -EPSILON) {
            float relSpeedSq = relVelX*relVelX + relVelY*relVelY;
            // Si la vitesse le long de la normale est significative par rapport à la vitesse totale
            if (velAlongNormal * velAlongNormal > 0.01f * relSpeedSq) {
                // Clamp timeToRewind pour éviter les bugs si ça explose
                if (timeToRewind > 0.05f) timeToRewind = 0.05f; 

                // REWIND : On recule les objets dans le temps
                pA.x -= pA.vx * timeToRewind;
                pA.y -= pA.vy * timeToRewind;
                pB.x -= pB.vx * timeToRewind;
                pB.y -= pB.vy * timeToRewind;
                solved = true;
            }
        }

        if (!solved) { // Fallback : Projection classique (Si collision statique ou trop lente)
            float move = result.depth / totalMass;
            pA.x += result.normal_vector.x * invMassA * move;
            pA.y += result.normal_vector.y * invMassA * move;
            pB.x -= result.normal_vector.x * invMassB * move;
            pB.y -= result.normal_vector.y * invMassB * move;
        }

        // Application de l'impulsion (Rebond)
        apply_impulse_response(pA, pB, result.normal_vector);
        
        // RE-FORWARD : Si on a rembobiné, on ré-avance avec les NOUVELLES vitesses
        if(solved){
            pA.x += pA.vx * timeToRewind;
            pA.y += pA.vy * timeToRewind;
            pB.x += pB.vx * timeToRewind;
            pB.y += pB.vy * timeToRewind;
        }
    }
}

void PhysicsSystem::solve_collisions() {
    // On itère sur le CACHE linéaire (Ultra rapide)
    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        if (pA.invMass == 0) continue; // Les murs ne cherchent pas

        int cx = (int)std::floor(pA.x / CELL_SIZE);
        int cy = (int)std::floor(pA.y / CELL_SIZE);

        // Voisins 3x3
        for(int dy = -1; dy <= 1; ++dy) {
            for(int dx = -1; dx <= 1; ++dx) {
                int hash = get_spatial_hash(cx + dx, cy + dy);
                const auto& cell = hash_grid[hash];

                for(int other_i : cell.particle_indices) {
                    if (i == other_i) continue;
                    ParticleCache& pB = cache[other_i];

                    // Optimisation "Processed Pairs" via ID (Seulement entre dynamiques)
                    if (pB.invMass > 0 && pA.entity >= pB.entity) continue;

                    // Broadphase Cache
                    float dist_x = std::abs(pA.x - pB.x);
                    float rSum = pA.r + pB.r;
                    if (dist_x > rSum) continue;
                    float dist_y = std::abs(pA.y - pB.y);
                    if (dist_y > rSum) continue;

                    // Résolution
                    resolve_pair(pA, pB);
                }
            }
        }
    }
}

RayHit PhysicsSystem::sweep_static_collisions(entt::registry& registry, 
                                              const Position& pos, 
                                              const Collider& body_collider, 
                                              float dx, float dy) {
    RayHit best_hit;
    best_hit.t = 1.0f;

    // Récupération de la taille du joueur (on suppose que c'est le rayon d'un cercle pour simplifier)
    // Si ton joueur est une boîte, prends la moitié de la largeur
    float body_radius = body_collider.radius; 
    if(body_collider.type == ColliderType::BOX) body_radius = std::min(body_collider.width, body_collider.height) / 2.0f;

    auto statics = registry.view<Position, Collider>(entt::exclude<Velocity>);

    statics.each([&](const auto& wall_pos, const auto& wall_col) {
        
        RayHit current_hit;

        if (wall_col.type == ColliderType::BOX) {
            // Cas 1 : Mur Carré (On gonfle la boîte avec le rayon du joueur)
            float expanded_w = wall_col.width + body_radius * 2.0f;
            float expanded_h = wall_col.height + body_radius * 2.0f;
            
            current_hit = ray_vs_aabb(pos.x, pos.y, dx, dy, 
                                      wall_pos.x, wall_pos.y, expanded_w, expanded_h);
        } 
        else if (wall_col.type == ColliderType::CIRCLE) {
            // Cas 2 : Mur Rond (On gonfle le cercle avec le rayon du joueur)
            float expanded_radius = wall_col.radius + body_radius;
            
            current_hit = ray_vs_circle(pos.x, pos.y, dx, dy, 
                                        wall_pos.x, wall_pos.y, expanded_radius);
        }

        if (current_hit.t < 0.0f) return;
        if (current_hit.hit && current_hit.t < best_hit.t) {
            best_hit = current_hit;
        }
    });

    return best_hit;
}

void PhysicsSystem::apply_continuous_movement(Position& pos, Velocity& vel, const RigidBody& body,
                                              const RayHit& hit, float dx, float dy, float dt) {
    if (hit.hit) {
        float safe_t = std::max(0.0f, hit.t - 0.001f);
        pos.x += dx * safe_t;
        pos.y += dy * safe_t;
        
        // Calcul manuel de la réponse impulsionnelle pour le CCD
        float dot = vel.x * hit.normal.x + vel.y * hit.normal.y;
        float j = -(1.0f + body.restitution * 0.5f) * dot; // Rebond contre mur statique
        
        vel.x += j * hit.normal.x;
        vel.y += j * hit.normal.y;

        float rem = 1.0f - hit.t;
        if (rem > 0) { pos.x += vel.x * dt * rem; pos.y += vel.y * dt * rem; }
    } else {
        pos.x += dx; pos.y += dy;
    }
}