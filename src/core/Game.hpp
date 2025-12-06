#pragma once

#include <entt/entt.hpp>
#include <SDL.h>
#include "physics/PhysicsSystem.hpp"

// Forward declaration
struct ImGuiIO;

class Game {
public:
    entt::registry registry;
    bool m_paused = false;
    bool m_step_once = false;

    Game();
    ~Game();

    bool init();
    void run();

private:
    void process_events();
    void update();
    void render();

    SDL_Window* window;
    SDL_Renderer* renderer;
    ImGuiIO* io;
    PhysicsSystem physics_system;
    bool is_running;
    int mouse_x, mouse_y;
    Uint64 last_tick = 0;
};
