#include "DebugSystem.hpp"
#include "core/Game.hpp"
#include <imgui.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>
#include "common/Components.hpp"
#include "physics/Components.hpp"

void render_debug_window(const ImGuiIO& io, const Game& game) {
    ImGui::Begin("Debug");
    ImGui::Text("Framerate: %.1f FPS", io.Framerate);
    ImGui::Text("Entities: %zu", game.registry.storage<entt::entity>()->size());

    if (game.m_paused) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "PAUSED");
    } else {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "RUNNING");
    }

    // --- Mechanical Energy Calculation ---
    float total_mechanical_energy = 0.0f;
    const float g = 980.0f;
    auto view = game.registry.view<const Position, const Velocity, const RigidBody, const Circle>();

    view.each([&total_mechanical_energy, &g](auto& pos, auto& vel, auto& body, auto& ){
        if (body.inverseMass > 0) {
            float mass = body.mass;
            float kinetic_energy = 0.5f * mass * (vel.x * vel.x + vel.y * vel.y);
            float potential_energy = mass * g * (500.0f - pos.y);
            total_mechanical_energy += kinetic_energy + potential_energy;
        }
    });
    total_mechanical_energy /= 1000.0f; // Convert to kJ
    ImGui::Text("Total Mechanical Energy: %.2f", total_mechanical_energy);
    // --- End ---

    ImGui::End();
    ImGui::ShowDemoWindow();
}

void handle_debug_input(Game& game, const SDL_Event& event) {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            case SDLK_SPACE:
                game.m_paused = !game.m_paused;
                game.time_scale = 1.0f;
                break;
            case SDLK_RIGHT:
                game.m_step_once = true;
                game.time_scale = 1.0f;
                break;
            case SDLK_LEFT:
                game.m_paused = true;
                game.m_step_once = true;
                game.time_scale = -1.0f;
                break;
        }
    }
}
