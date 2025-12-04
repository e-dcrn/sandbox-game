#include "Game.hpp"
#include "utils/Logger.hpp"
#include "components/Components.hpp"
#include "physics/PhysicsSystem.hpp"
#include "render/RenderSystem.hpp"
#include "render/DebugSystem.hpp"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <cstdlib>
#include <ctime>

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

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

    // Create Window
    window = SDL_CreateWindow("ECS Sandbox", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        log_error("Window could not be created", SDL_GetError());
        return false;
    }

    // Create Renderer
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
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

    // Create a single red circle entity
    auto entity = registry.create();
    registry.emplace<Position>(entity,
        static_cast<float>(SCREEN_WIDTH / 2),
        static_cast<float>(SCREEN_HEIGHT / 2)
    );
    registry.emplace<Velocity>(entity, 0.0f, 0.0f);
    registry.emplace<Circle>(entity, 
        30.0f, 
        SDL_Color{255, 0, 0, 255}
    );
    registry.emplace<FollowsMouse>(entity);

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
    update_velocity_system(registry, mouse_x, mouse_y);
    movement_system(registry);
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
