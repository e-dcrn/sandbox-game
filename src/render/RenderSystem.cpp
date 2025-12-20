#include "RenderSystem.hpp"
#include "../common/Components.hpp"
#include <SDL.h>
#include <cmath> 

SDL_Color get_contrast_color(SDL_Color c) {
    float luminance = 0.299f * c.r + 0.587f * c.g + 0.114f * c.b;
    return (luminance > 128) ? SDL_Color{0, 0, 0, 255} : SDL_Color{255, 255, 255, 255};
}

void draw_circle_pixels(SDL_Renderer* renderer, int cx, int cy, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}

void draw_circle_entity(SDL_Renderer* renderer, const Position& pos, const Circle& circle) {
    SDL_SetRenderDrawColor(renderer, circle.color.r, circle.color.g, circle.color.b, circle.color.a);
    draw_circle_pixels(renderer, static_cast<int>(pos.x), static_cast<int>(pos.y), static_cast<int>(circle.radius));

    SDL_Color lineColor = get_contrast_color(circle.color);
    SDL_SetRenderDrawColor(renderer, lineColor.r, lineColor.g, lineColor.b, 255);

    float end_x = pos.x + std::cos(pos.theta) * circle.radius;
    float end_y = pos.y + std::sin(pos.theta) * circle.radius;

    SDL_RenderDrawLine(renderer, 
                       static_cast<int>(pos.x), 
                       static_cast<int>(pos.y), 
                       static_cast<int>(end_x), 
                       static_cast<int>(end_y));
}

void draw_rectangle_entity(SDL_Renderer* renderer, const Position& pos, const Rectangle& rect) {
    float c = std::cos(pos.theta);
    float s = std::sin(pos.theta);
    float hw = rect.width * 0.5f;
    float hh = rect.height * 0.5f;

    float lx[4] = { -hw,  hw,  hw, -hw };
    float ly[4] = { -hh, -hh,  hh,  hh };

    SDL_Vertex vertices[4];

    for (int i = 0; i < 4; i++) {
        vertices[i].position.x = pos.x + (lx[i] * c - ly[i] * s);
        vertices[i].position.y = pos.y + (lx[i] * s + ly[i] * c);
        vertices[i].color = rect.color; 
        vertices[i].tex_coord = { 0.0f, 0.0f };
    }

    int indices[6] = { 0, 1, 2, 0, 2, 3 };

    SDL_RenderGeometry(renderer, nullptr, vertices, 4, indices, 6);
    
    SDL_Color lineColor = get_contrast_color(rect.color);
    SDL_SetRenderDrawColor(renderer, lineColor.r, lineColor.g, lineColor.b, 255);
    
    SDL_RenderDrawLine(renderer, 
        (int)pos.x, (int)pos.y, 
        (int)vertices[1].position.x, (int)vertices[1].position.y
    );
}


void render_game(SDL_Renderer* renderer, entt::registry& registry) {
    auto viewCircles = registry.view<const Position, const Circle>();
    viewCircles.each([&](const auto& pos, const auto& circle) {
        draw_circle_entity(renderer, pos, circle);
    });

    auto viewRects = registry.view<const Position, const Rectangle>();
    viewRects.each([&](const auto& pos, const auto& rect) {
        draw_rectangle_entity(renderer, pos, rect);
    });
}