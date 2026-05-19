#pragma once

#include <entt/entt.hpp>

struct SDL_Renderer;

void render_game(SDL_Renderer* renderer, entt::registry& registry);
