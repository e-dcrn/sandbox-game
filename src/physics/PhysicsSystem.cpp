#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"
#include "Collision.hpp"

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

        auto view = registry.view<RigidBody, Position, Velocity, ForceAccumulator>();
        view.each([sub_dt,time_scale](const auto& body, auto& pos, auto& vel, const auto& forces) {
            if (body.inverseMass == 0) return;

            float acc_x = forces.x * body.inverseMass;
            float acc_y = forces.y * body.inverseMass;

            vel.x += acc_x * sub_dt * time_scale;
            vel.y += acc_y * sub_dt * time_scale;
            if (vel.y > 1000.0f) vel.y = 1000.0f; 
            if (vel.y < -1000.0f) vel.y = -1000.0f;
            if (vel.x > 1000.0f) vel.x = 1000.0f; 
            if (vel.x < -1000.0f) vel.x = -1000.0f;
            pos.x += vel.x * sub_dt * time_scale;
            pos.y += vel.y * sub_dt * time_scale;
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

        relVelX = velB.x - velA.x;
        relVelY = velB.y - velA.y;
        velAlongNormal = relVelX * result.normal_vector.x + relVelY * result.normal_vector.y;

        float e = combine_restitution(bodyA.restitution, bodyB.restitution);
        if (velAlongNormal > -0.1f && velAlongNormal < 0.1f) {
            e = 0.0f;
        }
        float j = -(1.0f + e) * velAlongNormal;
        j /= totalMass;

        float impulseX = result.normal_vector.x * j;
        float impulseY = result.normal_vector.y * j;
        velA.x -= impulseX * invMassA;
        velA.y -= impulseY * invMassA;
        velB.x += impulseX * invMassB;
        velB.y += impulseY * invMassB;

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

    // 2. On itère sur les dynamiques
    // Note : On utilise 'each' pour capturer A, c'est le plus performant
    dynamics.each([&, this](entt::entity entity_A, Position& posA, Velocity& velA, const RigidBody& bodyA, const Collider& colA) {
        
        // On sauvegarde la position de A pour chercher les voisins
        // (Important pour ne pas chasser une cible mouvante si posA change)
        float startX = posA.x;
        float startY = posA.y;

        for (int cell_dx = -1; cell_dx <= 1; ++cell_dx) {
            for (int cell_dy = -1; cell_dy <= 1; ++cell_dy) {
                
                long key = get_hash_key(startX + cell_dx * CELL_SIZE, startY + cell_dy * CELL_SIZE);

                if (spatial_grid.count(key)) {
                    for (auto entity_B : spatial_grid.at(key)) {
                        
                        // Self-check
                        if (entity_A == entity_B) continue; 
                        
                        // FILTRE IMPORTANT : On ne veut que les Dynamiques ici 
                        // (car les Statiques sont gérés en Phase 1)
                        if (!registry.all_of<Velocity>(entity_B)) continue; 

                        // Vérification de la paire unique
                        entt::entity id1 = entity_A;
                        entt::entity id2 = entity_B;
                        if (id1 > id2) std::swap(id1, id2);
                        
                        long pair_id = ((long)id1 << 32) | (long)id2;
                        if (processed_pairs.count(pair_id)) continue; 
                        processed_pairs.insert(pair_id);

                        // Récupération de B
                        // Note : entity_B est validé par le check all_of<Velocity> au dessus
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