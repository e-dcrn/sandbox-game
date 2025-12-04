#pragma once

#include <SDL.h>

struct Position {
    float x, y;
};

struct Velocity {
    float x, y;
};

struct Circle {
    float radius;
    SDL_Color color;
};

struct FollowsMouse {};
