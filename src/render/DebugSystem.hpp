#pragma once

#include <entt/entt.hpp>

// Forward declaration
struct ImGuiIO;

void render_debug_window(const ImGuiIO& io, const entt::registry& registry);
