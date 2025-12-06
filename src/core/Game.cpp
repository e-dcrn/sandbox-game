#include "Game.hpp"
#include "utils/Logger.hpp"
#include "common/Components.hpp"
#include "physics/Components.hpp"
#include "render/RenderSystem.hpp"
#include "render/DebugSystem.hpp"
#include "EntityFactory.hpp"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <cstdlib>
#include <ctime>

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 612;

Game::Game() : window(nullptr), renderer(nullptr), io(nullptr), is_running(false), mouse_x(0), mouse_y(0) {
}

Game::~Game() {
    // Cleanup ImGui
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    // Cleanup SDL
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
    log_info("Cleanup complete.");
}

bool Game::init() {
    // Init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        log_error("SDL could not initialize", SDL_GetError());
        return false;
    }
    last_tick = SDL_GetTicks64();

    // Create Window
    window = SDL_CreateWindow("ECS Sandbox", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        log_error("Window could not be created", SDL_GetError());
        return false;
    }

    // Create Renderer
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED /*| SDL_RENDERER_PRESENTVSYNC*/);
    if (!renderer) {
        log_error("Renderer could not be created", SDL_GetError());
        return false;
    }

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    // Initialize Physics System
    physics_system.init();

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    
    Factory::create_wall(registry, 512.0f, 50.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 512.0f, 550.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 125.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_wall(registry, 899.0f, 300.0f, 100.0f, 500.0f);

    for (int i = 0; i < 200; ++i) {
        float randX = 200.0f + (rand() % 600); 
        float randY = 100.0f + (rand() % 200);
        Factory::create_ball(registry, randX, randY);
    }

    is_running = true;
    log_info("Game initialized successfully.");
    return true;
}

void Game::run() {
    while (is_running) {
        process_events();
        update();
        render();
    }
}

void Game::process_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT) {
            is_running = false;
        }
        if (event.type == SDL_MOUSEMOTION) {
            mouse_x = event.motion.x;
            mouse_y = event.motion.y;
        }
    }
}

void Game::update() {
    Uint64 current_tick = SDL_GetTicks64();
    float dt = (current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    // Slow down time step for stability of physics simulation
    if (dt > 0.05f) dt = 0.05f;
    
    physics_system.update(registry, dt);
}

void Game::render() {
    // Start ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Render debug window
    render_debug_window(*io, registry);

    // Clear screen
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    // Render game objects
    render_game(renderer, registry);

    // Render ImGui
    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    
    // Update screen
    SDL_RenderPresent(renderer);
}
