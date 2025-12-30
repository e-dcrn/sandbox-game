#pragma once
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include "EntityBuilder.hpp"
#include "../common/Components.hpp"

namespace Factory {

    inline entt::entity create_ball(entt::registry& registry, float x, float y) {
        return EntityBuilder(registry)
            .asCircle(15.0f)
            .at(x, y)
            .withColor(255, 0, 0)
            .withMass(1.0f)          // Masse 1kg -> Inertie calculée auto
            .withPhysicsMaterial(1.0f) // Ça rebondit bien
            .withCollider()
            .withGravity()
            .build();
    }

    inline entt::entity create_wall(entt::registry& registry, float x, float y, float w, float h) {
        return EntityBuilder(registry)
            .asBox(w, h)
            .at(x, y)
            .withColor(100, 100, 100)
            .withMass(0.0f)          // Statique -> Inertie 0 par défaut
            .withPhysicsMaterial(1.0f)
            .withCollider()
            // Pas de gravité
            .build();
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

    inline entt::entity create_spinner(entt::registry& registry, float x, float y, float w, float h, float speed_deg_per_sec) {
        return EntityBuilder(registry)
            .asBox(w, h)
            .at(x, y)
            .withColor(0, 255, 255)
            .withAngularVelocity(speed_deg_per_sec)
            .withMass(0.0f)
            .withInertiaMass(1.0f)  
            .withPhysicsMaterial(1.0f)
            .withCollider()
            .build();
    }
}