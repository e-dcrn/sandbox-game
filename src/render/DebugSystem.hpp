#pragma once

#include <entt/entt.hpp>
#include <SDL_events.h>

// Forward declaration
struct ImGuiIO;
class Game;

void render_debug_window(const ImGuiIO& io, Game& game);
void handle_debug_input(Game& game, const SDL_Event& event);
void take_snapshot(Game& game);

