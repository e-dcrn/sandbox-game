#include "DebugSystem.hpp"
#include "core/Game.hpp"
#include "utils/Serialization.hpp"

#include <imgui.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>
#include "common/Components.hpp"
#include "core/EntityFactory.hpp"
#include <cstdlib>
#include <ctime>


void render_debug_window(const ImGuiIO& io, Game& game) {
    ImGui::Begin("Debug");
    ImGui::Text("Framerate: %.1f FPS", io.Framerate);
    ImGui::Text("Entities: %zu", game.registry.storage<entt::entity>().size());
    ImGui::Text("History size: %zu", game.m_history.size());
    if (game.m_paused) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "PAUSED");
    } else {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "RUNNING");
    }

    float total_mechanical_energy = 0.0f;
    const float g = 980.0f;
    
    auto view = game.registry.view<const Position, const Velocity, const RigidBody>();

    view.each([&total_mechanical_energy, &g](auto& pos, auto& vel, auto& body){
        if (body.inverseMass > 0.0f) {
            float mass = body.mass; 
            float inertia = (body.inverseInertia > 0.0f) ? (1.0f / body.inverseInertia) : 0.0f;

            float kinetic_translation = 0.5f * mass * (vel.x * vel.x + vel.y * vel.y);
            float kinetic_rotation = 0.5f * inertia * (vel.omega * vel.omega);
            float potential_energy = mass * g * (500.0f - pos.y);

            total_mechanical_energy += kinetic_translation + kinetic_rotation + potential_energy;
        }
    });    
    total_mechanical_energy /= 1000.0f;
    ImGui::Text("Total Mechanical Energy: %.2f", total_mechanical_energy);

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
                if (game.m_paused) {
                    take_snapshot(game);
                }
                game.m_step_once = true;
                game.time_scale = 1.0f;
                break;
            case SDLK_LEFT:
                game.m_paused = true;
                if (!game.m_history.empty()) {
                    game.registry.clear();

                    std::stringstream storage(game.m_history.back(), std::ios::binary | std::ios::in | std::ios::out);
                    IArchive iarchive(storage);

                    entt::continuous_loader loader{game.registry};
                    loader.get<entt::entity>(iarchive)
                        .get<Position>(iarchive)
                        .get<Circle>(iarchive)
                        .get<Rectangle>(iarchive)
                        .get<FollowsMouse>(iarchive)
                        .get<RigidBody>(iarchive)
                        .get<ForceAccumulator>(iarchive)
                        .get<Velocity>(iarchive)
                        .get<GravityAffected>(iarchive)
                        .get<Collider>(iarchive)
                        .orphans();

                    game.m_history.pop_back();
                }
                break;
            case SDLK_r:
                game.reset();
                break;
        }
    }
}

void take_snapshot(Game& game) {
    std::stringstream storage(std::ios::binary | std::ios::out | std::ios::in);
    OArchive oarchive(storage);
    entt::snapshot{game.registry}
        .get<entt::entity>(oarchive)
        .get<Position>(oarchive)
        .get<Circle>(oarchive)
        .get<Rectangle>(oarchive)
        .get<FollowsMouse>(oarchive)
        .get<RigidBody>(oarchive)
        .get<ForceAccumulator>(oarchive)
        .get<Velocity>(oarchive)
        .get<GravityAffected>(oarchive)
        .get<Collider>(oarchive);
    game.m_history.push_back(storage.str());
    if (game.m_history.size() > Game::MAX_HISTORY_SIZE) {
        game.m_history.pop_front();
    }
}
