#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"

class PhysicsSystem {
public:
    void init(); // C'est ici qu'on va "charger" nos forces
    void update(entt::registry& registry, float dt);

private:
    ForceRegistry force_registry;
};