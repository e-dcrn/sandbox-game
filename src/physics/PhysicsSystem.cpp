#include "PhysicsSystem.hpp"
#include "../components/Components.hpp"
#include <cmath>

void update_velocity_system(entt::registry& registry, int mouse_x, int mouse_y) {
    // On demande <Position, Velocity, FollowsMouse>
    // MAIS comme FollowsMouse est vide, la lambda ne reçoit que (pos, vel)
    registry.view<Position, Velocity, FollowsMouse>().each([=](auto& pos, auto& vel) { 
        float dx = mouse_x - pos.x;
        float dy = mouse_y - pos.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance > 1.0f) {
            vel.x = dx / 10.0f;
            vel.y = dy / 10.0f;
        } else {
            vel.x = 0;
            vel.y = 0;
        }
    });
}

void movement_system(entt::registry& registry) {
    registry.view<Position, Velocity>().each([](auto& pos, const auto& vel) {
        pos.x += vel.x;
        pos.y += vel.y;
    });
}
