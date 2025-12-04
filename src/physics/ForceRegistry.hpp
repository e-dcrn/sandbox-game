#pragma once
#include <entt/entt.hpp>
#include <vector>
#include <functional>

using ForceFunction = std::function<void(entt::registry&, float)>;

class ForceRegistry {
public:
    void add(ForceFunction force_func) {
        forces.push_back(force_func);
    }

    void apply_all(entt::registry& registry, float dt) {
        for (auto& force : forces) {
            force(registry, dt);
        }
    }

    void clear() {
        forces.clear();
    }

private:
    std::vector<ForceFunction> forces;
};