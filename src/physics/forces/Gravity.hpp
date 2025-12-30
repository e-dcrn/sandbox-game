#pragma once
#include <entt/entt.hpp>
#include "../../common/Components.hpp"

namespace Forces {
    inline void apply_gravity(entt::registry& registry, float /*dt*/) { 
        auto view = registry.view<const RigidBody, ForceAccumulator, const GravityAffected>();
        
        view.each([](const RigidBody& body, ForceAccumulator& forces, const GravityAffected& grav) {
            if (body.inverseMass > 0) {
                float gravity_force = (1.0f / body.inverseMass) * 980.0f;
                forces.y += gravity_force * grav.scale; 
            }
        });
    }
}