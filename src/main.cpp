#include <SDL2/SDL.h>
#include <entt/entt.hpp>
#include <nlohmann/json.hpp>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <iostream> // Nécessaire pour afficher les erreurs dans le terminal (std::cerr)

// Petits codes couleurs pour le terminal (fonctionne sur Linux/Mac et Windows modernes)
#define COLOR_RESET "\033[0m"
#define COLOR_RED   "\033[31m"

// Fonction pour afficher les erreurs proprement
void log_error(const char* message) {
    std::cerr << COLOR_RED << "[ERREUR FATALE] " << message << " | Détails SDL: " << SDL_GetError() << COLOR_RESET << std::endl;
}

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

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    // 1. Init SDL (Check CRITIQUE)
    // Si SDL ne démarre pas, rien ne peut fonctionner.
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        log_error("Impossible d'initialiser la SDL");
        return -1;
    }

    SDL_WindowFlags flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("Sandbox Iso", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, flags);
    
    // 2. Check Window
    // Peut échouer si plus de RAM, ou pas de contexte graphique disponible.
    if (window == nullptr) {
        log_error("Impossible de créer la fenêtre");
        SDL_Quit(); // On nettoie ce qui a été fait avant (SDL_Init)
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, /*SDL_RENDERER_PRESENTVSYNC |*/ SDL_RENDERER_ACCELERATED);

    // 3. Check Renderer
    // Peut échouer si la carte graphique ne supporte pas l'accélération demandée.
    if (renderer == nullptr) {
        log_error("Impossible de créer le renderer");
        SDL_DestroyWindow(window); // On détruit la fenêtre créée juste avant
        SDL_Quit();
        return -1;
    }

    // 4. Init ImGui
    IMGUI_CHECKVERSION();
    if (ImGui::CreateContext() == nullptr) {
        std::cerr << COLOR_RED << "[ERREUR] Impossible de créer le contexte ImGui." << COLOR_RESET << std::endl;
        // Pas besoin de SDL_GetError ici, c'est interne à ImGui
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // // FPS Info
    // ImGuiIO& io = ImGui::GetIO(); (void)io; // Récupérer l'IO pour les infos (FPS, etc) 

    // Setup ImGui style
    // Vérification que les ponts entre SDL et ImGui s'initialisent bien
    if (!ImGui_ImplSDL2_InitForSDLRenderer(window, renderer)) {
        std::cerr << COLOR_RED << "[ERREUR] Échec init ImGui_SDL2." << COLOR_RESET << std::endl;
        return -1; 
    }
    
    if (!ImGui_ImplSDLRenderer2_Init(renderer)) {
        std::cerr << COLOR_RED << "[ERREUR] Échec init ImGui_SDLRenderer2." << COLOR_RESET << std::endl;
        return -1;
    }

    // 5. TEST ENTT
    entt::registry registry;
    auto entity = registry.create(); // EnTT crash rarement ici sauf si plus de RAM du tout.

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

        // // Fenêtre de debug
        // ImGui::Begin("Debug");
        // ImGui::Text("Entité ID: %d", (int)entity);
        // ImGui::Text("FPS: %.1f", io.Framerate);
        // ImGui::End();
        // ImGui::ShowDemoWindow();

        ImGui::Render();

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        
        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        draw_circle(renderer, mouseX, mouseY, 30, {255, 0, 0, 255});

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