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

    std::vector<entt::entity> entities(dynamics.begin(), dynamics.end());

    for (size_t i = 0; i < entities.size(); ++i) {
        for (size_t j = i + 1; j < entities.size(); ++j) {
            
            entt::entity entityA = entities[i];
            entt::entity entityB = entities[j];

            auto& posA = dynamics.get<Position>(entityA);
            auto& velA = dynamics.get<Velocity>(entityA);
            auto& bodyA = dynamics.get<RigidBody>(entityA);
            auto& colA = dynamics.get<Collider>(entityA);

            auto& posB = dynamics.get<Position>(entityB);
            auto& velB = dynamics.get<Velocity>(entityB);
            auto& bodyB = dynamics.get<RigidBody>(entityB);
            auto& colB = dynamics.get<Collider>(entityB);

            resolve_pair(posA, velA, bodyA, colA, 
                         posB, velB, bodyB, colB, time_scale);
        }
    }
}