#pragma once

#include <entt/entt.hpp>

// Forward declaration to avoid including SDL.h in the header
struct SDL_Renderer;

void render_game(SDL_Renderer* renderer, entt::registry& registry);
