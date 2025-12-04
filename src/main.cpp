#include <SDL2/SDL.h>
#include <entt/entt.hpp>
#include <nlohmann/json.hpp>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <iostream>
#include <cmath>

// Petits codes couleurs pour le terminal (fonctionne sur Linux/Mac et Windows modernes)
#define COLOR_RESET "\033[0m"
#define COLOR_RED   "\033[31m"

// Fonction pour afficher les erreurs proprement
void log_error(const char* message) {
    std::cerr << COLOR_RED << "[ERREUR FATALE] " << message << " | Détails SDL: " << SDL_GetError() << COLOR_RESET << std::endl;
}

// Components
struct Position {
    float x;
    float y;
};

struct Velocity {
    float dx;
    float dy;
};

struct Circle {
    int radius;
    SDL_Color color;
};

struct FollowsMouse {}; // Tag component

// Fonction pour dessiner un cercle
void draw_circle(SDL_Renderer* renderer, int x, int y, int radius, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx*dx + dy*dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}

// Systems
void update_velocity_system(entt::registry &registry) {
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);

    auto view = registry.view<const Position, Velocity, const FollowsMouse>();
    view.each([&](const Position &pos, Velocity &vel) {
        const float speed = 4.0f;
        float dirX = mouseX - pos.x;
        float dirY = mouseY - pos.y;
        float dist = std::sqrt(dirX * dirX + dirY * dirY);

        if (dist > 1.0f) {
            vel.dx = (dirX / dist) * speed;
            vel.dy = (dirY / dist) * speed;
        } else {
            vel.dx = 0;
            vel.dy = 0;
        }
    });
}

void movement_system(entt::registry &registry) {
    auto view = registry.view<Position, const Velocity>();
    view.each([](Position &pos, const Velocity &vel) {
        pos.x += vel.dx;
        pos.y += vel.dy;
    });
}

void render_circle_system(SDL_Renderer* renderer, entt::registry &registry) {
    auto view = registry.view<const Position, const Circle>();
    view.each([&](const Position &pos, const Circle &circle) {
        draw_circle(renderer, static_cast<int>(pos.x), static_cast<int>(pos.y), circle.radius, circle.color);
    });
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    // 1. Init SDL (Check CRITIQUE)
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        log_error("Impossible d'initialiser la SDL");
        return -1;
    }

    SDL_WindowFlags flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("Sandbox Iso", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, flags);
    
    if (window == nullptr) {
        log_error("Impossible de créer la fenêtre");
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, /*SDL_RENDERER_PRESENTVSYNC |*/ SDL_RENDERER_ACCELERATED);

    if (renderer == nullptr) {
        log_error("Impossible de créer le renderer");
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // 4. Init ImGui
    IMGUI_CHECKVERSION();
    if (ImGui::CreateContext() == nullptr) {
        std::cerr << COLOR_RED << "[ERREUR] Impossible de créer le contexte ImGui." << COLOR_RESET << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // FPS Info
    ImGuiIO& io = ImGui::GetIO(); (void)io; // Récupérer l'IO pour les infos (FPS, etc) 
    
    if (!ImGui_ImplSDL2_InitForSDLRenderer(window, renderer)) {
        std::cerr << COLOR_RED << "[ERREUR] Échec init ImGui_SDL2." << COLOR_RESET << std::endl;
        return -1; 
    }
    
    if (!ImGui_ImplSDLRenderer2_Init(renderer)) {
        std::cerr << COLOR_RED << "[ERREUR] Échec init ImGui_SDLRenderer2." << COLOR_RESET << std::endl;
        return -1;
    }

    entt::registry registry;
    auto entity = registry.create();
    auto circle_entity = registry.create();
    registry.emplace<Position>(circle_entity, 640.0f, 360.0f);
    registry.emplace<Velocity>(circle_entity, 0.0f, 0.0f);
    registry.emplace<Circle>(circle_entity, 30, SDL_Color{255, 0, 0, 255});
    registry.emplace<FollowsMouse>(circle_entity);

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) running = false;
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        
        // Fenêtre de debug
        ImGui::Begin("Debug");
        ImGui::Text("Entité ID: %d", (int)entity);
        ImGui::Text("FPS: %.1f", io.Framerate);
        ImGui::End();
        ImGui::ShowDemoWindow();

        ImGui::Render();

        // Game
        update_velocity_system(registry);
        movement_system(registry);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        
        render_circle_system(renderer, registry);

        // Fin Game

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    // Cleanup
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}