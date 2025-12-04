#include "PhysicsSystem.hpp"
#include "../common/Components.hpp"
#include "Components.hpp"
#include "forces/Gravity.hpp"

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });
}

void PhysicsSystem::update(entt::registry& registry, float dt) {
    registry.view<ForceAccumulator>().each([](auto& force) {
        force.x = 0; force.y = 0;
    });

    force_registry.apply_all(registry, dt);

    auto view = registry.view<RigidBody, Position, Velocity, ForceAccumulator>();
    view.each([dt](const auto& body, auto& pos, auto& vel, const auto& forces) {
        if (body.inverseMass == 0) return;

        float acc_x = forces.x * body.inverseMass;
        float acc_y = forces.y * body.inverseMass;

        vel.x += acc_x * dt;
        vel.y += acc_y * dt;
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        
        // Hack sol...
        if (pos.y > 612) { pos.y = 612; vel.y *= -0.6f; }
    });
}