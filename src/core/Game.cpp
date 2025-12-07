#include "Game.hpp"
#include "utils/Logger.hpp"
#include "common/Components.hpp"
#include "physics/Components.hpp"
#include "render/DebugSystem.hpp"
#include "EntityFactory.hpp"
#include "utils/Serialization.hpp"

// --- IMGUI OPENGL ---
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h> // Nouveau backend
#include <cstdlib>
#include <ctime>

// Inutile d'inclure glad ici si Renderer.hpp le fait, 
// mais on a besoin des constantes GL pour glClear
#include <glad/glad.h> 

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 612;

Game::Game() : window(nullptr), glContext(nullptr), io(nullptr), is_running(false), mouse_x(0), mouse_y(0) {
}

Game::~Game() {
    // 1. Cleanup Renderer (VBOs, VAOs)
    m_renderer.cleanup();

    // 2. Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown(); // Backend GL3
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    // 3. Cleanup SDL
    if (glContext) {
        SDL_GL_DeleteContext(glContext);
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

    // --- CONFIGURATION OPENGL (Avant de créer la fenêtre) ---
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    last_tick = SDL_GetTicks64();

    // Create Window avec le flag OPENGL
    window = SDL_CreateWindow("ECS Sandbox (GPU Instancing)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
        SCREEN_WIDTH, SCREEN_HEIGHT, 
        SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

    if (!window) {
        log_error("Window could not be created", SDL_GetError());
        return false;
    }

    // Create OpenGL Context
    glContext = SDL_GL_CreateContext(window);
    if (!glContext) {
        log_error("OpenGL Context could not be created", SDL_GetError());
        return false;
    }
    
    // Activer la VSync (1) ou désactiver (0) pour voir les max FPS
    SDL_GL_SetSwapInterval(0); 

    // --- INIT RENDERER (Charge GLAD) ---
    // Important de le faire avant ImGui
    m_renderer.init();

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    
    // Init ImGui pour SDL2 et OpenGL3
    ImGui_ImplSDL2_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initialize Physics System
    physics_system.init();

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    
    // Création de la scène
    Factory::create_wall(registry, 512.0f, 50.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 512.0f, 550.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 125.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_wall(registry, 899.0f, 300.0f, 100.0f, 500.0f);

    Factory::create_balls_in_circle(registry, 500.0f, 300.0f, 200.0f, 2000);

    is_running = true;
    log_info("Game initialized successfully [GPU Mode].");
    return true;
}

void Game::run() {
    while (is_running) {
        process_events();

        if (!m_paused) {
            take_snapshot(*this);
        }

        if (!m_paused || m_step_once) {
            update();
            m_step_once = false;
        }

        render();
    }
}

void Game::process_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        handle_debug_input(*this, event); // Assure-toi que cette fonction ne dépend pas de SDL_Renderer
        if (event.type == SDL_QUIT) {
            is_running = false;
        }
        if (event.type == SDL_MOUSEMOTION) {
            mouse_x = event.motion.x;
            mouse_y = event.motion.y;
        }
        // Gestion du redimensionnement fenêtre pour OpenGL
        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
            glViewport(0, 0, event.window.data1, event.window.data2);
        }
    }
}

void Game::update() {
    Uint64 current_tick = SDL_GetTicks64();
    float dt = (current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    if (dt > 0.01f) dt = 0.01f;
    
    // Update physique (Grille spatiale + Intégration)
    physics_system.update(registry, dt);
}

void Game::render() {
    // 1. Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // 2. ImGui Logic
    render_debug_window(*io, *this);

    // 3. OpenGL Clear
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Gris foncé
    glClear(GL_COLOR_BUFFER_BIT);

    // 4. Game Rendering (GPU Instancing)
    m_renderer.render(registry, w, h);

    // 5. Render ImGui over game
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
    // 6. Swap Buffers
    SDL_GL_SwapWindow(window);
}

void Game::reset() {
    registry.clear();
    m_history.clear();

    Factory::create_wall(registry, 512.0f, 50.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 512.0f, 550.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 125.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_wall(registry, 899.0f, 300.0f, 100.0f, 500.0f);

    Factory::create_balls_in_circle(registry, 500.0f, 300.0f, 200.0f, 2000);
}