#pragma once

#include <entt/entt.hpp>
#include <SDL.h>

// Forward declaration
struct ImGuiIO;

class Game {
public:
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
    entt::registry registry;
    ImGuiIO* io;
    bool is_running;
    int mouse_x, mouse_y;
};
