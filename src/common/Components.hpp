#pragma once

#include <SDL.h>

struct Position {
    float x;
    float y;
};

struct Circle {
    float radius;
    SDL_Color color;
};

struct FollowsMouse {};
