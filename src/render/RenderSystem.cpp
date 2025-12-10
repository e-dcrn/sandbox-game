#include "RenderSystem.hpp"
#include "../common/Components.hpp"
#include <SDL.h>
#include <cmath>

// -------------------------------------------------------------------------
// 1. Fonction pour dessiner un CERCLE (avec indicateur d'angle)
// -------------------------------------------------------------------------
void draw_circle(SDL_Renderer* renderer, float x, float y, float radius, SDL_Color color, float angle) {
    // A. Dessiner le disque plein (Pixel par pixel - lent mais simple)
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    
    int r = static_cast<int>(radius);
    int cx = static_cast<int>(x);
    int cy = static_cast<int>(y);

    for (int w = 0; w < r * 2; w++) {
        for (int h = 0; h < r * 2; h++) {
            int dx = r - w;
            int dy = r - h;
            if ((dx * dx + dy * dy) <= (r * r)) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }

    // B. Dessiner la ligne de rotation (Indicateur d'angle)
    // On dessine une ligne noire du centre vers l'extérieur
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    
    float end_x = x + std::cos(angle) * radius;
    float end_y = y + std::sin(angle) * radius;
    
    SDL_RenderDrawLine(renderer, cx, cy, static_cast<int>(end_x), static_cast<int>(end_y));
}

// -------------------------------------------------------------------------
// 2. Fonction pour dessiner un RECTANGLE/CARRÉ (avec rotation)
// -------------------------------------------------------------------------
void draw_square(SDL_Renderer* renderer, float x, float y, float w, float h, SDL_Color color, float angle) {
    
    // Cas Optimisé : Pas de rotation (Angle ~ 0)
    if (std::abs(angle) < 0.001f) {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect sdlRect;
        sdlRect.x = static_cast<int>(x - (w * 0.5f));
        sdlRect.y = static_cast<int>(y - (h * 0.5f));
        sdlRect.w = static_cast<int>(w);
        sdlRect.h = static_cast<int>(h);
        SDL_RenderFillRect(renderer, &sdlRect);
        return;
    }

    // Cas Rotation : Utilisation de la Géométrie (Triangles)
    float c = std::cos(angle);
    float s = std::sin(angle);
    float hw = w * 0.5f; // Half Width
    float hh = h * 0.5f; // Half Height

    // 4 coins locaux (centrés sur 0,0)
    // TL: Top-Left, TR: Top-Right, BR: Bottom-Right, BL: Bottom-Left
    float local_x[] = {-hw, hw, hw, -hw};
    float local_y[] = {-hh, -hh, hh, hh};

    SDL_Vertex vertices[4];

    for(int i = 0; i < 4; i++) {
        // Rotation 2D + Translation vers la position (x, y)
        vertices[i].position.x = x + (local_x[i] * c - local_y[i] * s);
        vertices[i].position.y = y + (local_x[i] * s + local_y[i] * c);
        
        vertices[i].color = color;
        vertices[i].tex_coord = {0.0f, 0.0f};
    }

    // Indices pour former 2 triangles (0-1-2 et 2-3-0)
    int indices[] = {0, 1, 2, 2, 3, 0};

    SDL_RenderGeometry(renderer, nullptr, vertices, 4, indices, 6);
}

// -------------------------------------------------------------------------
// 3. Fonction RENDER principale (Boucle sur les entités)
// -------------------------------------------------------------------------
void render_game(SDL_Renderer* renderer, entt::registry& registry) {
    
    // --- 1. Rendu des Cercles ---
    auto viewCircles = registry.view<const Position, const Circle>();
    viewCircles.each([&](auto entity, const auto& pos, const auto& circle) {
        float angle = 0.0f;
        
        // On récupère l'angle seulement si le composant Rotation existe
        if (registry.all_of<Rotation>(entity)) {
            angle = registry.get<Rotation>(entity).angle;
        }

        draw_circle(renderer, pos.x, pos.y, circle.radius, circle.color, angle);
    });

    // --- 2. Rendu des Rectangles ---
    auto viewRects = registry.view<const Position, const Rectangle>();
    viewRects.each([&](auto entity, const auto& pos, const auto& rect) {
        float angle = 0.0f;

        // On récupère l'angle seulement si le composant Rotation existe
        if (registry.all_of<Rotation>(entity)) {
            angle = registry.get<Rotation>(entity).angle;
        }

        draw_square(renderer, pos.x, pos.y, rect.width, rect.height, rect.color, angle);
    });
}