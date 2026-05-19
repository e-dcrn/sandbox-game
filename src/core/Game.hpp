#pragma once

#include <entt/entt.hpp>
#include <SDL.h>
#include "physics/PhysicsSystem.hpp"
#include <deque>
#include <string>
#include <sstream>

struct ImGuiIO;

class Game {
public:
    entt::registry registry;
    std::deque<std::string> m_history;
    static const size_t MAX_HISTORY_SIZE = 900; // 15 seconds at 60 FPS
    bool m_paused = false;
    bool m_step_once = false;
    float time_scale = 1.0f;

    Game();
    ~Game();

    bool init();
    void run();
    void reset();

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
