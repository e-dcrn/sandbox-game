#pragma once
#include <entt/entt.hpp>
#include "ForceRegistry.hpp"

class PhysicsSystem {
public:
    void init();
    void update(entt::registry& registry, float dt);

private:
    ForceRegistry force_registry;
};