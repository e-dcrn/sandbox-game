#include "Game.hpp"
#include "../utils/Logger.hpp"
#include "../common/Components.hpp"
#include "../physics/Components.hpp"
#include "render/DebugSystem.hpp"
#include "EntityFactory.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <glad/glad.h> 
#include <cstdlib>
#include <ctime>

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 612;

Game::Game() : window(nullptr), glContext(nullptr), io(nullptr), is_running(false), mouse_x(0), mouse_y(0) {}

Game::~Game() {
    m_renderer.cleanup();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    if (glContext) SDL_GL_DeleteContext(glContext);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
}

bool Game::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return false;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    window = SDL_CreateWindow("ECS Sandbox (GPU)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    glContext = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(0); // VSync OFF (Max FPS)

    m_renderer.init();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    physics_system.init();
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    
    // MURS
    Factory::create_wall(registry, 512.0f, 50.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 512.0f, 550.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 125.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_wall(registry, 899.0f, 300.0f, 100.0f, 500.0f);
    
    // BALLES (Fontaine)
    Factory::create_balls_in_circle(registry, 500.0f, 300.0f, 200.0f, 9000);

    is_running = true;
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
        handle_debug_input(*this, event);
        if (event.type == SDL_QUIT) is_running = false;
        if (event.type == SDL_MOUSEMOTION) {
            mouse_x = event.motion.x;
            mouse_y = event.motion.y;
        }
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
    physics_system.update(registry, dt);
}

void Game::render() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    render_debug_window(*io, *this);
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(255.0f, 255.0f, 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    m_renderer.render(registry, w, h);

    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    auto wall_view = registry.view<Position, Collider>();
    
    wall_view.each([&](const auto& pos, const auto& col) {
        if (col.type == ColliderType::BOX) {
            // ImGui utilise des coordonnées écran, parfait pour nous
            float x1 = pos.x - col.width * 0.5f;
            float y1 = pos.y - col.height * 0.5f;
            float x2 = pos.x + col.width * 0.5f;
            float y2 = pos.y + col.height * 0.5f;
            
            // Rectangle Rouge, rempli
            draw_list->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(200, 50, 50, 255));
        }
    });

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
}

void Game::reset() {
    registry.clear();
    m_history.clear();

    Factory::create_wall(registry, 512.0f, 50.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 512.0f, 550.0f, 824.0f, 100.0f);
    Factory::create_wall(registry, 125.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_wall(registry, 899.0f, 300.0f, 100.0f, 500.0f);
    Factory::create_balls_in_circle(registry, 500.0f, 300.0f, 200.0f, 9000);
}