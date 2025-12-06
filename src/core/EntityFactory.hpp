#pragma once
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include "../common/Components.hpp"
#include "../physics/Components.hpp"

namespace Factory {

    inline void add_physics(entt::registry& registry, entt::entity entity, float mass, bool gravity = true) {
        
        float inv_mass = (mass > 0) ? 1.0f / mass : 0.0f;
        
        registry.emplace<RigidBody>(entity, mass, inv_mass, 1.0f);
        registry.emplace<ForceAccumulator>(entity, 0.0f, 0.0f);

        if (gravity) registry.emplace<GravityAffected>(entity);
    }

    inline entt::entity create_ball(entt::registry& registry, float x, float y) {
        auto entity = registry.create();

        registry.emplace<Position>(entity, x, y);
        registry.emplace<Velocity>(entity, 0.0f, 0.0f);
        registry.emplace<Circle>(entity, 2.0f, SDL_Color{255, 0, 0, 255});
        registry.emplace<Collider>(entity, ColliderType::CIRCLE, 2.0f, 0.0f, 0.0f);

        add_physics(registry, entity, 1.0f, true);

        return entity;
    }

    inline entt::entity create_wall(entt::registry& registry, float x, float y, float w, float h) {
    auto entity = registry.create();
    
    registry.emplace<Position>(entity, x, y);
    registry.emplace<Rectangle>(entity, w, h, SDL_Color{100, 100, 100, 255}); // Gris
    registry.emplace<Collider>(entity, ColliderType::BOX, 0.0f, w, h);
    
    add_physics(registry, entity, 0.0f, false); 
    
    return entity;
}
}