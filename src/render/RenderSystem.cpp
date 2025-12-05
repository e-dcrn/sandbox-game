#include "RenderSystem.hpp"
#include "../common/Components.hpp"
#include <SDL.h>

// Helper function to draw a circle, not exposed in the header
void draw_circle(SDL_Renderer* renderer, int center_x, int center_y, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, center_x + dx, center_y + dy);
            }
        }
    }
}

void render_game(SDL_Renderer* renderer, entt::registry& registry) {
    auto view = registry.view<const Position, const Circle>();
    view.each([&](const auto& pos, const auto& circle) {
        SDL_SetRenderDrawColor(renderer, circle.color.r, circle.color.g, circle.color.b, circle.color.a);
        draw_circle(renderer, static_cast<int>(pos.x), static_cast<int>(pos.y), static_cast<int>(circle.radius));
    });

    auto viewRects = registry.view<const Position, const Rectangle>();
    viewRects.each([&](const auto& pos, const auto& rect) {
        SDL_SetRenderDrawColor(renderer, rect.color.r, rect.color.g, rect.color.b, rect.color.a);
        SDL_Rect sdlRect;
        sdlRect.x = static_cast<int>(pos.x - (rect.width * 0.5f));
        sdlRect.y = static_cast<int>(pos.y - (rect.height * 0.5f));
        sdlRect.w = static_cast<int>(rect.width);
        sdlRect.h = static_cast<int>(rect.height);
        SDL_RenderFillRect(renderer, &sdlRect);
    });
}
