#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });
}

void PhysicsSystem::update(entt::registry& registry, float dt, float time_scale) {

    const int sub_steps = 1;
    float sub_dt = dt / sub_steps;

    for(int i = 0; i < sub_steps; ++i){
        registry.view<ForceAccumulator>().each([](auto& force) {
            force.x = 0; force.y = 0;
        });

        force_registry.apply_all(registry, sub_dt);

        auto view = registry.view<RigidBody, Position, Velocity, Collider, ForceAccumulator>();
        view.each([&registry, sub_dt, time_scale, this](const auto& body, auto& pos, auto& vel, const auto& col, const auto& forces) {
            if (body.inverseMass == 0) return;

            float acc_x = forces.x * body.inverseMass;
            float acc_y = forces.y * body.inverseMass;
            vel.x += acc_x * sub_dt * time_scale;
            vel.y += acc_y * sub_dt * time_scale;
            float dx = vel.x * sub_dt * time_scale;
            float dy = vel.y * sub_dt * time_scale;

            RayHit wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            wall_hit.hit = false;
            apply_continuous_movement(pos, vel, body, wall_hit, dx, dy, sub_dt);
        });

        solve_collisions(registry,time_scale);
    }
}

long PhysicsSystem::get_hash_key(float x, float y) const {
    int cell_x = (int)std::floor(x / CELL_SIZE);
    int cell_y = (int)std::floor(y / CELL_SIZE);
    return ((long)cell_x << 32) | (long)cell_y;
}

void PhysicsSystem::build_grid(entt::registry& registry) {
    spatial_grid.clear();
    auto view = registry.view<Position, Velocity>();
    
    view.each([&](auto entity, const auto& pos, const auto& /*vel*/) {
        long key = get_hash_key(pos.x, pos.y);
        spatial_grid[key].push_back(entity);
    });
}

float PhysicsSystem::combine_restitution(float rA, float rB) {
    if (rA > 1.0f || rB > 1.0f) return std::max(rA, rB);
    return rA * rB;
}

// Cette fonction ne touche PAS aux positions (pos.x), seulement aux vitesses (vel.x)
void PhysicsSystem::apply_impulse_response(
    Velocity& velA, float invMassA,
    Velocity& velB, float invMassB,
    const RigidBody& bodyA, const RigidBody& bodyB,
    const Vect2& normal, float time_scale) 
{
    // 1. Calcul de la vitesse relative
    float relVelX = velA.x - velB.x;
    float relVelY = velA.y - velB.y;
    
    // 2. Vitesse le long de la normale
    float velAlongNormal = relVelX * normal.x + relVelY * normal.y;

    // Si les objets s'éloignent déjà, on ne fait rien
    if (velAlongNormal > 0) return;

    // 3. Calcul de la restitution (rebond)
    float e = combine_restitution(bodyA.restitution, bodyB.restitution);
    
    // Seuil anti-vibration (si la collision est très lente, pas de rebond)
    if (velAlongNormal > -0.1f && velAlongNormal < 0.1f) {
        e = 0.0f;
    }

    // 4. Calcul de l'impulsion scalaire j
    float j = -(1.0f + e) * velAlongNormal;
    float totalMass = invMassA + invMassB;
    if (totalMass > 0) j /= totalMass;

    // 5. Application de l'impulsion sur les vitesses
    float impulseX = normal.x * j;
    float impulseY = normal.y * j;

    velA.x += impulseX * invMassA * time_scale; // A recule
    velA.y += impulseY * invMassA * time_scale;
    
    velB.x -= impulseX * invMassB * time_scale; // B avance
    velB.y -= impulseY * invMassB * time_scale;
}

void PhysicsSystem::resolve_pair(
    Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA,
    Position& posB, Velocity& velB, const RigidBody& bodyB, const Collider& colB,
    float time_scale
) {
    CollisionResult result = {false, {0,0}, 0};

    if (colA.type == ColliderType::CIRCLE && colB.type == ColliderType::CIRCLE) {
        result = check_circle_circle({posA.x, posA.y}, colA.radius, {posB.x, posB.y}, colB.radius);
    }
    else if (colA.type == ColliderType::CIRCLE && colB.type == ColliderType::BOX) {
        result = check_circle_box({posA.x, posA.y}, colA.radius, {posB.x, posB.y}, {colB.width, colB.height});
    }
    else if (colA.type == ColliderType::BOX && colB.type == ColliderType::CIRCLE) {
        result = check_circle_box({posB.x, posB.y}, colB.radius, {posA.x, posA.y}, {colA.width, colA.height});
        result.normal_vector.x *= -1.0f;
        result.normal_vector.y *= -1.0f;
    }
    else {
        result = check_box_box({posA.x, posA.y}, {colA.width, colA.height}, {posB.x, posB.y}, {colB.width, colB.height});
    }

    if (result.is_colliding) {
        float invMassA = bodyA.inverseMass;
        float invMassB = bodyB.inverseMass;
        float totalMass = invMassA + invMassB;

        if (totalMass == 0.0f) return;

        float relVelX = (velA.x - velB.x) * time_scale;
        float relVelY = (velA.y - velB.y) * time_scale;
        float velAlongNormal = relVelX * result.normal_vector.x + relVelY * result.normal_vector.y;
        
        if (velAlongNormal > 0) return;

        bool solved = false;
        float timeToRewind = result.depth / -velAlongNormal;
        const float EPSILON = 0.001f;
        if (velAlongNormal < -EPSILON) {
            float relSpeedSq = relVelX*relVelX + relVelY*relVelY;
            if (velAlongNormal * velAlongNormal > 0.01f * relSpeedSq) {
                if (timeToRewind > 0.05f) timeToRewind = 0.05f;

                posA.x -= velA.x * timeToRewind * time_scale;
                posA.y -= velA.y * timeToRewind * time_scale;
                posB.x -= velB.x * timeToRewind * time_scale;
                posB.y -= velB.y * timeToRewind * time_scale;
                solved = true;
            }
        }

        if (!solved) { // Old version fallback
            float move = result.depth / totalMass;
            posA.x += result.normal_vector.x * invMassA * move;
            posA.y += result.normal_vector.y * invMassA * move;
            posB.x -= result.normal_vector.x * invMassB * move;
            posB.y -= result.normal_vector.y * invMassB * move;
        }

        apply_impulse_response(velA, bodyA.inverseMass, 
                               velB, bodyB.inverseMass, 
                               bodyA, bodyB, result.normal_vector, time_scale);
        
        if(solved){
            posA.x += velA.x * timeToRewind * time_scale;
            posA.y += velA.y * timeToRewind * time_scale;
            posB.x += velB.x * timeToRewind * time_scale;
            posB.y += velB.y * timeToRewind * time_scale;
        }
    }
}

void PhysicsSystem::solve_collisions(entt::registry& registry, float time_scale) {
    auto dynamics = registry.view<Position, Velocity, RigidBody, Collider>();
    auto statics = registry.view<Position, RigidBody, Collider>(entt::exclude<Velocity>);

    dynamics.each([&statics,time_scale](Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA) {
        statics.each([&](const Position& posB, const RigidBody& bodyB, const Collider& colB) {
            Velocity staticVel = {0.0f, 0.0f}; 
            resolve_pair(posA, velA, bodyA, colA, 
                         const_cast<Position&>(posB), staticVel, bodyB, colB, time_scale); 
        });
    });

    build_grid(registry);
    std::unordered_set<long> processed_pairs;

    dynamics.each([&, this](entt::entity entity_A, Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA) {
        float startX = posA.x;
        float startY = posA.y;
        for (int cell_dx = -1; cell_dx <= 1; ++cell_dx) {
            for (int cell_dy = -1; cell_dy <= 1; ++cell_dy) {
                
                long key = get_hash_key(startX + cell_dx * CELL_SIZE, startY + cell_dy * CELL_SIZE);
                if (spatial_grid.count(key)) {
                    for (auto entity_B : spatial_grid.at(key)) {
                        if (entity_A == entity_B) continue; 
                        if (!registry.all_of<Velocity>(entity_B)) continue; 

                        entt::entity id1 = entity_A;
                        entt::entity id2 = entity_B;
                        if (id1 > id2) std::swap(id1, id2);
                        
                        long pair_id = ((long)id1 << 32) | (long)id2;
                        if (processed_pairs.count(pair_id)) continue; 
                        processed_pairs.insert(pair_id);

                        auto& posB = registry.get<Position>(entity_B);
                        auto& velB = registry.get<Velocity>(entity_B);
                        const auto& bodyB = registry.get<RigidBody>(entity_B);
                        const auto& colB = registry.get<Collider>(entity_B);

                        resolve_pair(
                            posA, velA, bodyA, colA, 
                            posB, velB, bodyB, colB, 
                            time_scale
                        );
                    }
                }
            }
        }
    });
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
        // --- ETAPE 1 : GESTION POSITION (Spécifique Continu) ---
        // On avance juste au bon endroit
        float safe_t = std::max(0.0f, hit.t - 0.001f);
        pos.x += dx * safe_t;
        pos.y += dy * safe_t;

        // --- ETAPE 2 : REPONSE PHYSIQUE (Partagée) ---
        // On crée un faux corps statique pour le mur
        Velocity wallVel = {0,0};
        RigidBody wallBody; 
        wallBody.inverseMass = 0; // Masse infinie
        wallBody.restitution = 0.5f; 

        // On réutilise la même logique de rebond !
        apply_impulse_response(vel, body.inverseMass, 
                               wallVel, 0.0f, 
                               body, wallBody, {hit.normal.x, hit.normal.y}, 1.0f);

        float remaining_t = 1.0f - hit.t;

        // S'il reste du temps significatif, on bouge avec la NOUVELLE vitesse
        if (remaining_t > 0.0f) {
            // Distance = Vitesse_Actuelle * Temps_Total_Frame * Portion_Restante
            pos.x += vel.x * dt * remaining_t;
            pos.y += vel.y * dt * remaining_t;
        }        
    } else {
        pos.x += dx;
        pos.y += dy;
    }
}