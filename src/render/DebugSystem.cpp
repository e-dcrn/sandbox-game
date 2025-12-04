#include "DebugSystem.hpp"
#include <imgui.h>

void render_debug_window(const ImGuiIO& io, const entt::registry& registry) {
    ImGui::Begin("Debug");
    ImGui::Text("Framerate: %.1f FPS", io.Framerate);
    ImGui::Text("Entities: %zu", registry.storage<entt::entity>()->size());
    ImGui::End();
    ImGui::ShowDemoWindow();
}
