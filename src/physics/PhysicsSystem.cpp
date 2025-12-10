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

        auto view = registry.view<RigidBody, Position, Velocity, Rotation, Collider, ForceAccumulator>();
        view.each([&registry, sub_dt, this](auto entity, const auto& body, auto& pos, auto& vel, auto& rot, const auto& col, const auto& forces) {
            // if (body.inverseMass == 0) return;

            float acc_x = forces.x * body.inverseMass;
            float acc_y = forces.y * body.inverseMass;
            vel.x += acc_x * sub_dt;
            vel.y += acc_y * sub_dt;
            float dx = vel.x * sub_dt;
            float dy = vel.y * sub_dt;
            rot.angle += rot.omega * sub_dt;

            // RayHit wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            RayHit wall_hit;
            wall_hit.hit = false;
            if (registry.any_of<UseRaycast>(entity)) {
                wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            }
            apply_continuous_movement(pos, vel, body, wall_hit, dx, dy, sub_dt);
        });

        build_cache_from_registry(registry);
        for(int j = 0; j < 6; ++j){
            solve_collisions(dt);
        }
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

void PhysicsSystem::build_cache_from_registry(entt::registry& registry) {
    cache.clear(); // On vide le cache, mais pas la grille (fait ailleurs)
    
    auto view = registry.view<Position, Collider>();
    
    view.each([this, &registry](auto entity, const auto& pos, const auto& col) {
        float vx = 0, vy = 0;
        float ax = 0, ay = 0;
        float invMass = 0, restitution = 1.0f, invInertia = 0.0f;
        float angle = 0.0f, omega = 0.0f;

        if (registry.all_of<RigidBody>(entity)) {
            const auto& body = registry.get<RigidBody>(entity);
            invMass = body.inverseMass;
            restitution = body.restitution;
            invInertia = body.invInertia;

            if (invMass > 0 && registry.all_of<ForceAccumulator>(entity)) {
                const auto& forces = registry.get<ForceAccumulator>(entity);
                ax = forces.x * invMass;
                ay = forces.y * invMass;
            }
        }
        if (registry.all_of<Velocity>(entity)) {
            const auto& vel = registry.get<Velocity>(entity);
            vx = vel.x; vy = vel.y;
        }
        if (registry.all_of<Rotation>(entity)) {
            const auto& rot = registry.get<Rotation>(entity);
            angle = rot.angle;
            omega = rot.omega;
        }

        int type = (col.type == ColliderType::CIRCLE) ? 0 : 1;
        float hw = col.width * 0.5f;
        float hh = col.height * 0.5f;
        
        // Rayon effectif pour le broadphase (Diagonale pour les boites)
        float r = (type == 0) ? col.radius : std::sqrt(hw*hw + hh*hh); 

        // On remplit le cache
        cache.push_back({
            entity, 
            pos.x, pos.y, 
            vx, vy, 
            ax, ay,
            angle, omega, invInertia,
            r, invMass, restitution, 
            type, hw, hh
        });
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
        if (p.invMass > 0) {
            auto& pos = registry.get<Position>(p.entity);
            auto& vel = registry.get<Velocity>(p.entity);
            pos.x = p.x;
            pos.y = p.y;
            vel.x = p.vx;
            vel.y = p.vy;
        }
        if (p.invInertia > 0 || p.invMass > 0) {
            if (registry.all_of<Rotation>(p.entity)) {
                auto& rot = registry.get<Rotation>(p.entity);
                rot.angle = p.angle;
                rot.omega = p.omega; // C'est ici qu'on sauvegarde le rebond angulaire !
            }
        }
    }
}

float PhysicsSystem::combine_restitution(float rA, float rB) {
    if (rA > 1.0f || rB > 1.0f) return std::max(rA, rB);
    return rA * rB;
}

float cross(Vect2 a, Vect2 b) { return a.x * b.y - a.y * b.x; }
Vect2 cross(float s, Vect2 a) { return {-s * a.y, s * a.x}; }

void PhysicsSystem::apply_impulse_response(ParticleCache& pA, ParticleCache& pB, const CollisionResult& res) {
    
    // 1. Vecteurs "Bras de levier" (du Centre au Contact)
    Vect2 rA = { res.contact_point.x - pA.x, res.contact_point.y - pA.y };
    Vect2 rB = { res.contact_point.x - pB.x, res.contact_point.y - pB.y };

    // 2. Vitesse relative au POINT DE CONTACT
    // V_point = V_linear + Omega x Radius
    Vect2 vA_point = { pA.vx - pA.omega * rA.y, pA.vy + pA.omega * rA.x };
    Vect2 vB_point = { pB.vx - pB.omega * rB.y, pB.vy + pB.omega * rB.x };
    
    float rv_x = vA_point.x - vB_point.x;
    float rv_y = vA_point.y - vB_point.y;

    // 3. Vitesse le long de la normale
    float velAlongNormal = rv_x * res.normal_vector.x + rv_y * res.normal_vector.y;

    if (velAlongNormal > 0) return; // S'éloignent déjà

    // 4. Calcul du dénominateur (Masse inverse effective)
    float rAxN = cross(rA, res.normal_vector); // rA cross Normal
    float rBxN = cross(rB, res.normal_vector); // rB cross Normal
    
    float invMassSum = pA.invMass + pB.invMass 
                     + (rAxN * rAxN) * pA.invInertia 
                     + (rBxN * rBxN) * pB.invInertia;

    if (invMassSum == 0.0f) return;

    // 5. Impulsion Scalaire
    float e = combine_restitution(pA.restitution, pB.restitution);
    float j = -(1.0f + e) * velAlongNormal;
    j /= invMassSum;

    // 6. Application
    Vect2 impulse = { res.normal_vector.x * j, res.normal_vector.y * j };

    // Update Linéaire
    pA.vx += impulse.x * pA.invMass;
    pA.vy += impulse.y * pA.invMass;
    pB.vx -= impulse.x * pB.invMass;
    pB.vy -= impulse.y * pB.invMass;

    // Update Angulaire : dOmega = (r x Impulse) / Inertia
    pA.omega += cross(rA, impulse) * pA.invInertia;
    pB.omega -= cross(rB, impulse) * pB.invInertia;
}

void PhysicsSystem::resolve_pair(ParticleCache& pA, ParticleCache& pB, float dt) {
    CollisionResult result = {false, {0,0}, 0, {0,0}};

    // --- 1. DÉTECTION (Adaptation des données Cache vers tes fonctions de check) ---
    // Note : p.hw et p.hh sont les demi-largeurs, tes fonctions attendent souvent width/height (x2)
    
    if (pA.type == 0 && pB.type == 0) { // Circle vs Circle
        result = check_circle_circle({pA.x, pA.y}, pA.r, {pB.x, pB.y}, pB.r);
    }
    else if (pA.type == 0 && pB.type == 1) { // Circle vs Box
        result = check_circle_rotated_box({pA.x, pA.y}, pA.r, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.angle);
    }
    else if (pA.type == 1 && pB.type == 0) { // Box vs Circle
        result = check_circle_rotated_box({pB.x, pB.y}, pB.r, {pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.angle);
        // Inversion normale car A vs B
        result.normal_vector.x *= -1.0f;
        result.normal_vector.y *= -1.0f;
    }
    else { // Box vs Box
        result = check_box_box({pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f});
    }

    // --- 2. RÉSOLUTION (Ton Algorithme Exact) ---

    if (result.is_colliding) {
        // 1. Vitesse Relative au POINT DE CONTACT (Crucial : Linéaire + Angulaire)
        // On doit utiliser la vitesse du point précis, sinon le seuil sera faux pour un objet qui tourne.
        Vect2 rA = { result.contact_point.x - pA.x, result.contact_point.y - pA.y };
        Vect2 rB = { result.contact_point.x - pB.x, result.contact_point.y - pB.y };

        Vect2 vA_point = { pA.vx - pA.omega * rA.y, pA.vy + pA.omega * rA.x };
        Vect2 vB_point = { pB.vx - pB.omega * rB.y, pB.vy + pB.omega * rB.x };

        float rv_x = vA_point.x - vB_point.x;
        float rv_y = vA_point.y - vB_point.y;
        float velAlongNormal = rv_x * result.normal_vector.x + rv_y * result.normal_vector.y;

        // 2. Accélération Relative (Linéaire suffit ici)
        // On regarde si les forces (gravité, moteur...) poussent les objets l'un vers l'autre
        float relAccX = pA.ax - pB.ax;
        float relAccY = pA.ay - pB.ay;
        float accAlongNormal = relAccX * result.normal_vector.x + relAccY * result.normal_vector.y;

        // 3. Définition du Seuil Dynamique
        // La vitesse gagnée uniquement grâce aux forces sur cette frame
        float closingSpeedFromGravity = accAlongNormal * dt;

        // On ajoute une micro marge (k_slop) pour absorber les erreurs flottantes
        const float k_slop = 0.001f;

        // CONDITION : Est-ce un vrai crash ?
        // Si on se rapproche plus vite que ce que la gravité seule ferait -> IMPACT
        bool is_real_impact = velAlongNormal < (closingSpeedFromGravity - k_slop);

        if (is_real_impact) {
            // === CAS 1 : IMPACT VIOLENT ===
            // C'est ici qu'on fait le REWIND car la vitesse est significative
            
            // Calcul sécurisé du Rewind
            float timeToRewind = 0.0f;
            if (velAlongNormal < -0.0001f) {
                timeToRewind = result.depth / -velAlongNormal;
            }
            
            // Clamp timeToRewind entre 0 et dt (indispensable)
            if (timeToRewind > dt) timeToRewind = dt;
            if (timeToRewind < 0.0f) timeToRewind = 0.0f;

            pA.x -= pA.vx * timeToRewind; pA.y -= pA.vy * timeToRewind; pA.angle -= pA.omega * timeToRewind;
            pB.x -= pB.vx * timeToRewind; pB.y -= pB.vy * timeToRewind; pB.angle -= pB.omega * timeToRewind;

            // 4. ADAPTATION DU POINT DE CONTACT (Le Fix de l'Energie)
            // Le point de contact doit aussi reculer, sinon le bras de levier est faux.
            // On approxime son mouvement par la vitesse du système au point de contact.
            CollisionResult impactRes = result;
            impactRes.depth = 0; // À l'impact, depth est nulle
            
            // On déplace le point de contact en arrière selon sa propre vitesse
            // On moyenne les vitesses des deux points pour estimer le déplacement du contact
            // (Ou on prend vA si B est statique)
            float vContactX = vA_point.x; // Simplification acceptable
            float vContactY = vA_point.y;
            // Si tu veux être précis : vContact = (vA_point * invMA + vB_point * invMB) / (invMA + invMB)
            
            impactRes.contact_point.x -= vContactX * timeToRewind;
            impactRes.contact_point.y -= vContactY * timeToRewind;

            // 5. RESOLUTION
            apply_impulse_response(pA, pB, impactRes);

            // 6. FORWARD (On avance avec les NOUVELLES vitesses)
            pA.x += pA.vx * timeToRewind; pA.y += pA.vy * timeToRewind; pA.angle += pA.omega * timeToRewind;
            pB.x += pB.vx * timeToRewind; pB.y += pB.vy * timeToRewind; pB.angle += pB.omega * timeToRewind;
        } 
        else {
            // === CAS 2 : CONTACT AU REPOS (RESTING) ===
            // Ici, la vitesse relative est faible (proche de 0 ou causée juste par la gravité).
            // On ne fait PAS de rewind (trop instable pour de si petites valeurs).
            
            // A. On résout l'impulse normalement
            // Astuce : Pour le resting, on force souvent la restitution (rebond) à 0
            // pour éviter que les objets ne vibrent sur le sol.
            CollisionResult restingRes = result;
            apply_impulse_response(pA, pB, restingRes); 

            // B. On fait juste la projection (Position Correction) pour ne pas s'enfoncer
            float totalInvMass = pA.invMass + pB.invMass;
            if (totalInvMass > 0.0f) {
                float move = result.depth / totalInvMass;
                // On peut appliquer un "Baumgarte stabilization" ici (corriger seulement 20-50% de l'erreur)
                // move *= 0.5f; 
                
                pA.x += result.normal_vector.x * pA.invMass * move;
                pA.y += result.normal_vector.y * pA.invMass * move;
                pB.x -= result.normal_vector.x * pB.invMass * move;
                pB.y -= result.normal_vector.y * pB.invMass * move;
            }
        }
    }
}

void PhysicsSystem::solve_collisions(float dt) {
    refresh_grid_from_cache();
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
                    resolve_pair(pA, pB,dt);
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