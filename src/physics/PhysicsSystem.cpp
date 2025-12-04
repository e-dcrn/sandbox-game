#include "PhysicsSystem.hpp"
#include "../common/Components.hpp"
#include "Components.hpp"
#include "forces/Gravity.hpp"

void update_physics(entt::registry& registry, float dt) {
    
    registry.view<ForceAccumulator>().each([](auto& force) {
        force.x = 0; force.y = 0;
    });

    Forces::apply_gravity(registry, 500.0f);

    auto view = registry.view<RigidBody, Position, Velocity, ForceAccumulator>();
    view.each([dt](const auto& body, auto& pos, auto& vel, const auto& forces) {
        if (body.inverseMass == 0) return;

        // a = F / m
        float acc_x = forces.x * body.inverseMass;
        float acc_y = forces.y * body.inverseMass;

        // v += a * dt
        vel.x += acc_x * dt;
        vel.y += acc_y * dt;

        // p += v * dt
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;

        // Hack temporaire : Sol à 700px
        if (pos.y > 612) {
            pos.y = 612;
            vel.y = -vel.y * 0.6f; // Rebond amorti
        }
    });
}