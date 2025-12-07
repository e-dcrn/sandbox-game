#pragma once
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include "../common/Components.hpp"
#include "../physics/Components.hpp"

namespace Factory {

    inline void add_physics(entt::registry& registry, entt::entity entity, float mass, float bounce = 1.0f, bool gravity = true) {
        
        float inv_mass = (mass > 0) ? 1.0f / mass : 0.0f;
        
        registry.emplace<RigidBody>(entity, mass, inv_mass, bounce);
        registry.emplace<ForceAccumulator>(entity, 0.0f, 0.0f);

        if (gravity) registry.emplace<GravityAffected>(entity);
    }

    inline entt::entity create_ball(entt::registry& registry, float x, float y) {
        auto entity = registry.create();

        registry.emplace<Position>(entity, x, y);
        registry.emplace<Velocity>(entity, 0.0f, 0.0f);
        registry.emplace<Circle>(entity, 3.0f, SDL_Color{255, 0, 0, 255});
        registry.emplace<Collider>(entity, ColliderType::CIRCLE, 3.0f, 0.0f, 0.0f);

        add_physics(registry, entity, 1.0f, 0.90f, true);

        return entity;
    }

    inline entt::entity create_wall(entt::registry& registry, float x, float y, float w, float h) {
    auto entity = registry.create();
    
    registry.emplace<Position>(entity, x, y);
    registry.emplace<Rectangle>(entity, w, h, SDL_Color{100, 100, 100, 255}); // Gris
    registry.emplace<Collider>(entity, ColliderType::BOX, 0.0f, w, h);
    
    add_physics(registry, entity, 0.0f, 1.0f, false); 
    
    return entity;
}

    inline void create_balls_in_circle(entt::registry& registry, float centerX, float centerY, float maxRadius, int count) {
        for (int i = 0; i < count; ++i) {
            float angle = ((rand() % 6283) / 1000.0f);
            float random0to1 = (rand() % 1000) / 1000.0f;
            float distance = maxRadius * std::sqrt(random0to1);
            float randX = centerX + std::cos(angle) * distance;
            float randY = centerY + std::sin(angle) * distance;

            create_ball(registry, randX, randY);
        }
    }
}