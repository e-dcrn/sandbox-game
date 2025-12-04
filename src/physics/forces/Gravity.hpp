#pragma once
#include <entt/entt.hpp>
#include "../Components.hpp"

namespace Forces {
    inline void apply_gravity(entt::registry& registry, float gravity_y = 980.0f) { 
        auto view = registry.view<const RigidBody, ForceAccumulator>();
        
        view.each([gravity_y](const RigidBody& body, ForceAccumulator& forces) {
            if (body.inverseMass > 0) {
                forces.y += (1.0f / body.inverseMass) * gravity_y; 
            }
        });
    }
}