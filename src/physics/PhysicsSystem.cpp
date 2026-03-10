#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"

constexpr int PHYSICS_SUB_STEPS = 1;
constexpr float REWIND_THRESHOLD_SQ = 0.00001f;
constexpr float MAX_REWIND_TIME = 0.05f;
constexpr float EPSILON = 0.00001f;

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });

    hash_grid.resize(HASH_SIZE);
    for(auto& cell : hash_grid) cell.particle_indices.reserve(16);
    cache.reserve(3000);
}

void PhysicsSystem::update(entt::registry& registry, float dt) {

    float sub_dt = dt / PHYSICS_SUB_STEPS;

    for(int i = 0; i < PHYSICS_SUB_STEPS; ++i){
        registry.view<ForceAccumulator>().each([](auto& force) {
            force.x = 0; force.y = 0;
        });

        force_registry.apply_all(registry, sub_dt);

        auto view = registry.view<RigidBody, Position, Velocity, Collider, ForceAccumulator>();
        view.each([&registry, sub_dt, this](auto entity, const auto& body, auto& pos, auto& vel, const auto& col, const auto& forces) {
            if (body.inverseMass == 0 && body.inverseInertia == 0 && vel.omega == 0 && vel.x == 0 && vel.y == 0) return;
            
            Vect2 acceleration = Vect2{forces.x, forces.y} * body.inverseMass;
            vel.x += acceleration.x * sub_dt;
            vel.y += acceleration.y * sub_dt;
            pos.theta += vel.omega * sub_dt;
            float dx = vel.x * sub_dt;
            float dy = vel.y * sub_dt;

            // RaycastHit wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            Collision::RaycastHit wall_hit;
            wall_hit.hit = false;
            if (registry.any_of<UseRaycast>(entity)) {
                wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            }
            apply_continuous_movement(pos, vel, body, wall_hit, dx, dy, sub_dt);
        });

        sync_from_registry(registry);
        for(int j = 0; j < 6; ++j){
            solve_collisions();
        }
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

void PhysicsSystem::sync_from_registry(entt::registry& registry) {
    cache.clear(); // On vide le cache, mais pas la grille (fait ailleurs)
    
    auto view = registry.view<Position, Collider>();
    
    view.each([this, &registry](auto entity, const auto& pos, const auto& col) {
        float vx = 0, vy = 0, omega = 0, invMass = 0, restitution = 1.0f, invInertia = 0.0f;

        if (registry.all_of<RigidBody>(entity)) {
            const auto& body = registry.get<RigidBody>(entity);
            invMass = body.inverseMass;
            restitution = body.restitution;
            invInertia = body.inverseInertia;
        }
        if (registry.all_of<Velocity>(entity)) {
            const auto& vel = registry.get<Velocity>(entity);
            vx = vel.x; vy = vel.y; omega = vel.omega;
        }

        int type = (col.type == ColliderType::CIRCLE) ? 0 : 1;
        float hw = col.width * 0.5f;
        float hh = col.height * 0.5f;

        cache.push_back({entity, pos.x, pos.y, pos.theta, vx, vy, omega, col.radius, invMass, invInertia, restitution, type, hw, hh});
    });
}

void PhysicsSystem::refresh_grid_from_cache() {
    for(auto& cell : hash_grid) cell.particle_indices.clear();

    for(int i = 0; i < (int)cache.size(); ++i) {
        const auto& p = cache[i];

        Vect2 aabb = PhysicsMath::compute_aabb_half_dims(p.type, p.r, p.hw, p.hh, p.theta);
        auto bounds = PhysicsMath::compute_grid_bounds(p.x, p.y, aabb.x, aabb.y, CELL_SIZE);

        for (int y = bounds.startY; y <= bounds.endY; ++y) {
            for (int x = bounds.startX; x <= bounds.endX; ++x) {
                int hash = get_spatial_hash(x, y);
                hash_grid[hash].particle_indices.push_back(i);
            }
        }
    }
}

void PhysicsSystem::apply_changes_to_registry(entt::registry& registry) {
    // Copie Cache -> Registry (Seulement pour les dynamiques)
    for(const auto& p : cache) {
        if (p.invMass > 0 || p.invInertia > 0) {
            auto& pos = registry.get<Position>(p.entity);
            auto& vel = registry.get<Velocity>(p.entity);
            
            pos = { p.x, p.y, p.theta };
            vel = { p.vx, p.vy, p.omega };
        }
    }
}

// Retourne le vecteur d'impulsion (J * normale) sans l'appliquer
Vect2 PhysicsSystem::calculate_impulse_vector(const ParticleCache& pA, const ParticleCache& pB, 
        const Vect2& normal, const Vect2& contact_point){
    
    Vect2 rA = contact_point - Vect2{pA.x, pA.y};
    Vect2 rB = contact_point - Vect2{pB.x, pB.y};

    // Ton fix pour les cercles
    if (pA.type == 0) rA = normal * pA.r;
    if (pB.type == 0) rB = -normal * pB.r;

    Vect2 velA = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA);
    Vect2 velB = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB);
    Vect2 relVel = velB - velA;

    float velAlongNormal = PhysicsMath::dot(relVel, normal);
    
    // Si les objets s'éloignent, aucune impulsion requise
    if (velAlongNormal > 0) return {0.0f, 0.0f};

    float e = PhysicsMath::combine_restitution(pA.restitution, pB.restitution);

    float numerateur = -(1.0f + e) * velAlongNormal;
    float denominateur = PhysicsMath::calculate_total_impulse_denominator(
        pA.invMass, pA.invInertia, rA, 
        pB.invMass, pB.invInertia, rB, normal);

    if (denominateur == 0.0f) return {0.0f, 0.0f};

    float j = numerateur / denominateur;
    
    // On retourne le vecteur impulsion complet
    return normal * j;
}

void PhysicsSystem::solve_sequential_impulse(ParticleCache& pA, ParticleCache& pB, const Collision::Manifold& result) {
    
    for (int i = 0; i < result.contact_count; ++i) {
        Vect2 cp = result.contacts[i];
        
        Vect2 rA = cp - Vect2{pA.x, pA.y};
        Vect2 rB = cp - Vect2{pB.x, pB.y};
        
        if (pA.type == 0) rA = result.normal_vector * pA.r;
        if (pB.type == 0) rB = -result.normal_vector * pB.r;

        Vect2 velA = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA);
        Vect2 velB = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB);
        Vect2 relVel = velB - velA;
        
        float velAlongNormal = PhysicsMath::dot(relVel, result.normal_vector);
        if (velAlongNormal >= 0) continue;
        float e = PhysicsMath::combine_restitution(pA.restitution, pB.restitution);
        if (std::abs(velAlongNormal) < 20.0f) e = 0.0f; 

        float numerateur = -(1.0f + e) * velAlongNormal;
        float denominateur = PhysicsMath::calculate_total_impulse_denominator(
            pA.invMass, pA.invInertia, rA, 
            pB.invMass, pB.invInertia, rB, 
            result.normal_vector
        );

        if (denominateur > 0.0f) {
            float j = numerateur / denominateur;

            Vect2 impulse = result.normal_vector * j;
            pA.vx -= impulse.x * pA.invMass;
            pA.vy -= impulse.y * pA.invMass;
            pA.omega -= (rA.x * impulse.y - rA.y * impulse.x) * pA.invInertia;

            pB.vx += impulse.x * pB.invMass;
            pB.vy += impulse.y * pB.invMass;
            pB.omega += (rB.x * impulse.y - rB.y * impulse.x) * pB.invInertia;
        }
    }
}

void PhysicsSystem::resolve_contact(ParticleCache& pA, ParticleCache& pB) {
    Collision::Manifold result = {false, {0,0}, 0, 0, {{0,0}, {0,0}}};

    // --- Détection (Inchangé) ---
    if (pA.type == 0 && pB.type == 0) { 
        result = Collision::test_circles({pA.x, pA.y}, pA.r, {pB.x, pB.y}, pB.r);
    }
    else if (pA.type == 0 && pB.type == 1) { 
        result = Collision::test_circle_box_rotated({pA.x, pA.y}, pA.r, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
        result.normal_vector = -result.normal_vector;
    }
    else if (pA.type == 1 && pB.type == 0) { 
        result = Collision::test_circle_box_rotated({pB.x, pB.y}, pB.r, {pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta);
    }
    else { 
        result = Collision::test_boxes_rotated({pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
    }

    if (result.is_colliding) {
        float totalMass = pA.invMass + pB.invMass;
        if (totalMass == 0.0f && pA.invInertia == 0.0f && pB.invInertia == 0.0f) return;

        // Pré-calcul Vitesse Impact
        Vect2 mainContact = result.contacts[0];
        Vect2 rA_check = mainContact - Vect2{pA.x, pA.y};
        Vect2 rB_check = mainContact - Vect2{pB.x, pB.y};
        Vect2 vA = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA_check);
        Vect2 vB = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB_check);
        float vNormal = PhysicsMath::dot(vB - vA, result.normal_vector);

        const float IMPACT_THRESHOLD = 100.0f;

        // --- BRANCHE 1 : REWIND (Haute vitesse OU Vibration forte) ---
        if (vNormal < -IMPACT_THRESHOLD) {
            float timeToRewind = result.depth / -vNormal;
            if (timeToRewind > MAX_REWIND_TIME) timeToRewind = MAX_REWIND_TIME; 

            // 1. Rewind
            PhysicsMath::integrate_position(pA.x, pA.y, pA.theta, pA.vx, pA.vy, pA.omega, -timeToRewind);
            PhysicsMath::integrate_position(pB.x, pB.y, pB.theta, pB.vx, pB.vy, pB.omega, -timeToRewind);

            // 2. Résolution (Appel Fonction)
            solve_sequential_impulse(pA, pB, result);

            // 3. Forward
            PhysicsMath::integrate_position(pA.x, pA.y, pA.theta, pA.vx, pA.vy, pA.omega, timeToRewind);
            PhysicsMath::integrate_position(pB.x, pB.y, pB.theta, pB.vx, pB.vy, pB.omega, timeToRewind);
        } 
        
        // --- BRANCHE 2 : REPOS (Correction de position + Résolution) ---
        else {
            const float SLOP = 0.5f;
            const float BAUMGARTE = 0.2f;

            // 1. Correction de Position (Slop)
            float penetration = std::max(0.0f, result.depth - SLOP);
            
            if (penetration > 0.0f) {
                float correctionMag = (penetration / totalMass) * BAUMGARTE;
                correctionMag /= (float)result.contact_count; 
                
                Vect2 correction = result.normal_vector * correctionMag;

                // --- AJOUT : KILLER DE MICRO-DRIFT DE POSITION ---
                // Si on doit corriger la position de moins de 0.01 pixel sur un axe, 
                // on considère que c'est du bruit numérique et on ne bouge pas cet axe.
                const float POS_CORRECTION_THRESHOLD = 0.05f; // Ajuste si nécessaire (0.01 à 0.05)

                if (std::abs(correction.x) < POS_CORRECTION_THRESHOLD) correction.x = 0.0f;
                if (std::abs(correction.y) < POS_CORRECTION_THRESHOLD) correction.y = 0.0f;
                // ------------------------------------------------

                pA.x -= correction.x * pA.invMass; pA.y -= correction.y * pA.invMass;
                pB.x += correction.x * pB.invMass; pB.y += correction.y * pB.invMass;
            }

            // 2. Résolution (Appel Fonction) - Reste inchangé
            solve_sequential_impulse(pA, pB, result);
        }
    }
}

void PhysicsSystem::solve_collisions() {
    refresh_grid_from_cache();
    // On itère sur le CACHE linéaire (Ultra rapide)
    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        if (pA.invMass == 0 && pA.invInertia == 0) continue; // Les murs ne cherchent pas

        int cx, cy;
        PhysicsMath::world_to_grid(pA.x, pA.y, CELL_SIZE, cx, cy);

        // Voisins 3x3
        for(int dy = -1; dy <= 1; ++dy) {
            for(int dx = -1; dx <= 1; ++dx) {
                int hash = get_spatial_hash(cx + dx, cy + dy);
                const auto& cell = hash_grid[hash];

                for(int other_i : cell.particle_indices) {
                    if (i == other_i) continue;
                    ParticleCache& pB = cache[other_i];

                    // Optimisation "Processed Pairs" via ID (Seulement entre dynamiques)
                    if ((pB.invMass > 0 || pB.invInertia > 0) && pA.entity >= pB.entity) continue;

                    // Résolution
                    resolve_contact(pA, pB);
                }
            }
        }
    }
}

Collision::RaycastHit PhysicsSystem::sweep_static_collisions(entt::registry& registry, 
                                              const Position& pos, 
                                              const Collider& body_collider, 
                                              float dx, float dy) {
    Collision::RaycastHit best_hit;
    best_hit.t = 1.0f;

    float body_radius = body_collider.radius; 
    if(body_collider.type == ColliderType::BOX) body_radius = std::min(body_collider.width, body_collider.height) / 2.0f;

    auto statics = registry.view<Position, Collider>(entt::exclude<Velocity>);

    statics.each([&](const auto& wall_pos, const auto& wall_col) {
        
        Collision::RaycastHit current_hit;

        if (wall_col.type == ColliderType::BOX) {
            float expanded_w = wall_col.width + body_radius * 2.0f;
            float expanded_h = wall_col.height + body_radius * 2.0f;
            
            current_hit = Collision::raycast_box_rotated(pos.x, pos.y, dx, dy, 
                                      wall_pos.x, wall_pos.y, expanded_w, expanded_h, wall_pos.theta);
        } 
        else if (wall_col.type == ColliderType::CIRCLE) {
            float expanded_radius = wall_col.radius + body_radius;
            
            current_hit = Collision::raycast_circle(pos.x, pos.y, dx, dy, 
                                        wall_pos.x, wall_pos.y, expanded_radius);
        }

        if (current_hit.t < 0.0f) return;
        if (current_hit.hit && current_hit.t < best_hit.t) {
            best_hit = current_hit;
        }Parfait. Maintenant pour fini, depuis que mes box peuvent tourner mon raycast ne marche plus (je pense) bien
    });

    return best_hit;
}

void PhysicsSystem::apply_continuous_movement(Position& pos, Velocity& vel, const RigidBody& body,
                                              const Collision::RaycastHit& hit, float dx, float dy, float dt) {
    if (!hit.hit) {
        pos.x += dx; 
        pos.y += dy;
        return;
    }

    float safe_t = std::max(0.0f, hit.t - 0.001f);
    pos.x += dx * safe_t;
    pos.y += dy * safe_t;
    
    float velAlongNormal = PhysicsMath::dot({vel.x, vel.y}, hit.normal);
    
    if (velAlongNormal < 0) {
        vel.x -= velAlongNormal * hit.normal.x;
        vel.y -= velAlongNormal * hit.normal.y;
    }
}