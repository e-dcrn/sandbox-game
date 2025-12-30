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
            
            float acc_x = forces.x * body.inverseMass;
            float acc_y = forces.y * body.inverseMass;
            vel.x += acc_x * sub_dt;
            vel.y += acc_y * sub_dt;
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
        
        // Rayon effectif pour le broadphase (Diagonale pour les boites)
        float r = (type == 0) ? col.radius : std::sqrt(hw*hw + hh*hh); 

        // On remplit le cache
        cache.push_back({entity, pos.x, pos.y, pos.theta, vx, vy, omega, r, invMass, invInertia, restitution, type, hw, hh});
    });
}

void PhysicsSystem::refresh_grid_from_cache() {
    for(auto& cell : hash_grid) cell.particle_indices.clear();

    for(int i = 0; i < (int)cache.size(); ++i) {
        const auto& p = cache[i];

        float obj_hw = (p.type == 0) ? p.r : p.hw;
        float obj_hh = (p.type == 0) ? p.r : p.hh;

        int startX = (int)std::floor((p.x - obj_hw) / CELL_SIZE);
        int endX   = (int)std::floor((p.x + obj_hw) / CELL_SIZE);
        int startY = (int)std::floor((p.y - obj_hh) / CELL_SIZE);
        int endY   = (int)std::floor((p.y + obj_hh) / CELL_SIZE);

        for (int y = startY; y <= endY; ++y) {
            for (int x = startX; x <= endX; ++x) {
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
            pos.x = p.x;
            pos.y = p.y;
            pos.theta = p.theta; 
            vel.x = p.vx;
            vel.y = p.vy;
            vel.omega = p.omega;
        }
    }
}

// Cette fonction ne touche PAS aux positions (pos.x), seulement aux vitesses (vel.x)
void PhysicsSystem::apply_impulse(ParticleCache& pA, ParticleCache& pB, 
        const Vect2& normal, const Vect2& contact_point) {
    Vect2 rA;
    Vect2 rB;

    if (pA.type == 0) { 
        // Le bras de levier rA pointe du centre vers le contact
        // C'est donc la normale (qui va de A vers B) multipliée par le rayon
        rA = { normal.x * pA.r, normal.y * pA.r };
    } else {
        rA = { contact_point.x - pA.x, contact_point.y - pA.y };
    }

    // Pour un cercle (Objet B)
    if (pB.type == 0) {
        // La normale va de A vers B, donc pour B le levier est inversé
        rB = { -normal.x * pB.r, -normal.y * pB.r };
    } else {
        rB = { contact_point.x - pB.x, contact_point.y - pB.y };
    }

    Vect2 rotVelA = PhysicsMath::cross(pA.omega, rA);
    Vect2 rotVelB = PhysicsMath::cross(pB.omega, rB);

    Vect2 totalVelA = { pA.vx + rotVelA.x, pA.vy + rotVelA.y };
    Vect2 totalVelB = { pB.vx + rotVelB.x, pB.vy + rotVelB.y };

    Vect2 relVel = { totalVelB.x - totalVelA.x, totalVelB.y - totalVelA.y };

    float velAlongNormal = relVel.x * normal.x + relVel.y * normal.y;

    if (velAlongNormal > 0) return;

    float e = PhysicsMath::combine_restitution(pA.restitution, pB.restitution);
    
    if (velAlongNormal > -EPSILON && velAlongNormal < EPSILON) e = 0.0f;
    float numerateur = -(1.0f + e) * velAlongNormal;

    float rA_cross_n = PhysicsMath::cross(rA, normal);
    float rB_cross_n = PhysicsMath::cross(rB, normal);

    float invMassSum = pA.invMass + pB.invMass;
    float angularCompA = (rA_cross_n * rA_cross_n) * pA.invInertia;
    float angularCompB = (rB_cross_n * rB_cross_n) * pB.invInertia;

    float denominateur = invMassSum + angularCompA + angularCompB;

    if (denominateur == 0.0f) return;

    float j = numerateur / denominateur;

    Vect2 impulse = { normal.x * j, normal.y * j };

    pA.vx -= impulse.x * pA.invMass;
    pA.vy -= impulse.y * pA.invMass;
    pA.omega -= pA.invInertia * PhysicsMath::cross(rA, impulse); 

    pB.vx += impulse.x * pB.invMass;
    pB.vy += impulse.y * pB.invMass;
    pB.omega += pB.invInertia * PhysicsMath::cross(rB, impulse);
}

void PhysicsSystem::resolve_contact(ParticleCache& pA, ParticleCache& pB) {
    Collision::Manifold result = {false, {0,0}, 0, {0,0}};

    if (pA.type == 0 && pB.type == 0) { // Circle vs Circle
        result = Collision::test_circles({pA.x, pA.y}, pA.r, {pB.x, pB.y}, pB.r);
    }
    else if (pA.type == 0 && pB.type == 1) { // Circle vs Box
        result = Collision::test_circle_box_rotated({pA.x, pA.y}, pA.r, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
        // Inversion normale car A vs B
        result.normal_vector.x *= -1.0f;
        result.normal_vector.y *= -1.0f;
    }
    else if (pA.type == 1 && pB.type == 0) { // Box vs Circle
        result = Collision::test_circle_box_rotated({pB.x, pB.y}, pB.r, {pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta);
    }
    else { // Box vs Box
        result = Collision::test_boxes_rotated({pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
    }

    if (result.is_colliding) {
        float invMassA = pA.invMass;
        float invMassB = pB.invMass;
        float totalMass = invMassA + invMassB;

        if (totalMass == 0.0f && pA.invInertia == 0.0f && pB.invInertia == 0.0f) return;

        Vect2 rA = { result.contact_point.x - pA.x, result.contact_point.y - pA.y };
        Vect2 rB = { result.contact_point.x - pB.x, result.contact_point.y - pB.y };

        Vect2 rotVelA = { -pA.omega * rA.y, pA.omega * rA.x };
        Vect2 rotVelB = { -pB.omega * rB.y, pB.omega * rB.x };

        Vect2 totalVelA = { pA.vx + rotVelA.x, pA.vy + rotVelA.y };
        Vect2 totalVelB = { pB.vx + rotVelB.x, pB.vy + rotVelB.y };

        Vect2 relVelContact = { totalVelB.x - totalVelA.x, totalVelB.y - totalVelA.y };
        float contactVelAlongNormal = relVelContact.x * result.normal_vector.x + relVelContact.y * result.normal_vector.y;

        if (contactVelAlongNormal < -EPSILON) {
            float timeToRewind = result.depth / -contactVelAlongNormal;
            if (timeToRewind > MAX_REWIND_TIME) timeToRewind = MAX_REWIND_TIME; 

            pA.x -= pA.vx * timeToRewind;
            pA.y -= pA.vy * timeToRewind;
            pA.theta -= pA.omega * timeToRewind;
            pB.x -= pB.vx * timeToRewind;
            pB.y -= pB.vy * timeToRewind;
            pB.theta -= pB.omega * timeToRewind;

            apply_impulse(pA, pB, result.normal_vector,result.contact_point);

            pA.x += pA.vx * timeToRewind;
            pA.y += pA.vy * timeToRewind;
            pA.theta += pA.omega * timeToRewind;
            pB.x += pB.vx * timeToRewind;
            pB.y += pB.vy * timeToRewind;
            pB.theta += pB.omega * timeToRewind;
        } 
        else {
            float move = result.depth / totalMass;
            pA.x -= result.normal_vector.x * invMassA * move;
            pA.y -= result.normal_vector.y * invMassA * move;
            pB.x += result.normal_vector.x * invMassB * move;
            pB.y += result.normal_vector.y * invMassB * move;

            apply_impulse(pA, pB, result.normal_vector,result.contact_point);
        }
    }
}

void PhysicsSystem::solve_collisions() {
    refresh_grid_from_cache();
    // On itère sur le CACHE linéaire (Ultra rapide)
    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        if (pA.invMass == 0 && pA.invInertia == 0) continue; // Les murs ne cherchent pas

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

    // Récupération de la taille du joueur (on suppose que c'est le rayon d'un cercle pour simplifier)
    // Si ton joueur est une boîte, prends la moitié de la largeur
    float body_radius = body_collider.radius; 
    if(body_collider.type == ColliderType::BOX) body_radius = std::min(body_collider.width, body_collider.height) / 2.0f;

    auto statics = registry.view<Position, Collider>(entt::exclude<Velocity>);

    statics.each([&](const auto& wall_pos, const auto& wall_col) {
        
        Collision::RaycastHit current_hit;

        if (wall_col.type == ColliderType::BOX) {
            // Cas 1 : Mur Carré (On gonfle la boîte avec le rayon du joueur)
            float expanded_w = wall_col.width + body_radius * 2.0f;
            float expanded_h = wall_col.height + body_radius * 2.0f;
            
            current_hit = Collision::raycast_box(pos.x, pos.y, dx, dy, 
                                      wall_pos.x, wall_pos.y, expanded_w, expanded_h);
        } 
        else if (wall_col.type == ColliderType::CIRCLE) {
            // Cas 2 : Mur Rond (On gonfle le cercle avec le rayon du joueur)
            float expanded_radius = wall_col.radius + body_radius;
            
            current_hit = Collision::raycast_circle(pos.x, pos.y, dx, dy, 
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
                                              const Collision::RaycastHit& hit, float dx, float dy, float dt) {
    if (!hit.hit) {
        pos.x += dx; 
        pos.y += dy;
        return;
    }

    float safe_t = std::max(0.0f, hit.t - 0.001f);
    pos.x += dx * safe_t;
    pos.y += dy * safe_t;
    
    float dot = vel.x * hit.normal.x + vel.y * hit.normal.y;
    
    float restitution_factor = 1.0f + body.restitution * 0.5f;
    float j = -restitution_factor * dot; 
    
    vel.x += j * hit.normal.x;
    vel.y += j * hit.normal.y;

    float rem = 1.0f - hit.t;
    if (rem > 0) { 
        pos.x += vel.x * dt * rem; 
        pos.y += vel.y * dt * rem; 
    }
}