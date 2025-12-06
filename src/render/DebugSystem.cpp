#include "DebugSystem.hpp"
#include "core/Game.hpp"
#include <imgui.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>

void render_debug_window(const ImGuiIO& io, const Game& game) {
    ImGui::Begin("Debug");
    ImGui::Text("Framerate: %.1f FPS", io.Framerate);
    ImGui::Text("Entities: %zu", game.registry.storage<entt::entity>()->size());

    if (game.m_paused) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "PAUSED");
    } else {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "RUNNING");
    }

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
