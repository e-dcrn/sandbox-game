#pragma once

#include <cmath>
#include <algorithm>

struct Vect2 {
    float x;
    float y;
};

struct CollisionResult {
    bool is_colliding;
    Vect2 normal_vector;
    float depth;
};

CollisionResult check_circle_circle(Vect2 centerA, float radiusA, Vect2 centerB, float radiusB) {
    float distX = centerA.x - centerB.x;
    float distY = centerA.y - centerB.y;
    float required_distance = radiusA + radiusB;
    
    float square_dist = (distX * distX) + (distY * distY);
    float required_square = required_distance * required_distance;
    
    if (square_dist < required_square) {
        float dist = std::sqrt(square_dist);
        if (dist == 0.0f) {
            return { true, {0.0f, 1.0f}, required_distance}; 
        }
        float inverse_dist = 1.0f / dist;

        return {
            true,
            {distX * inverse_dist, distY * inverse_dist},
            required_distance - dist 
        };
    }
    return {false, {0.0f, 0.0f}, 0.0f};
}

CollisionResult check_circle_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions) {
    float dx = circle_center.x - box_center.x;
    float dy = circle_center.y - box_center.y;
    float halfW = box_dimensions.x * 0.5f;
    float halfH = box_dimensions.y * 0.5f;

    float closestX = std::clamp(dx, -halfW, halfW);
    float closestY = std::clamp(dy, -halfH, halfH);

    bool inside = (dx == closestX && dy == closestY);

    if (inside) {
        float distToEdgeX = halfW - std::abs(dx);
        float distToEdgeY = halfH - std::abs(dy);

        Vect2 normal;
        float depth;

        if (distToEdgeX < distToEdgeY) {
            normal = { (dx > 0) ? 1.0f : -1.0f, 0.0f };
            depth = distToEdgeX + circle_radius;
        } 
        else {
            normal = { 0.0f, (dy > 0) ? 1.0f : -1.0f };
            depth = distToEdgeY + circle_radius;
        }

        return { true, normal, depth };
    }
    else {
        float diffX = dx - closestX;
        float diffY = dy - closestY;
        float distSq = diffX * diffX + diffY * diffY;

        if (distSq > circle_radius * circle_radius) {
            return {false, {0.0f, 0.0f}, 0.0f};
        }

        float dist = std::sqrt(distSq);
        Vect2 normal;
        
        if (dist == 0.0f) {
            normal = {0.0f, 1.0f}; 
        } else {
            normal = { diffX / dist, diffY / dist };
        }

        return { true, normal, circle_radius - dist };
    }
}

CollisionResult check_box_box(Vect2 centerA, Vect2 dimA, Vect2 centerB, Vect2 dimB) {
    float halfW_A = dimA.x * 0.5f;
    float halfH_A = dimA.y * 0.5f;
    float halfW_B = dimB.x * 0.5f;
    float halfH_B = dimB.y * 0.5f;

    float deltaX = centerB.x - centerA.x;
    float deltaY = centerB.y - centerA.y;
    float overlapX = halfW_A + halfW_B - std::abs(deltaX);
    float overlapY = halfH_A + halfH_B - std::abs(deltaY);

    if (overlapX > 0 && overlapY > 0) {
        if (overlapX < overlapY) {
            return {
                true,
                { (deltaX < 0) ? 1.0f : -1.0f, 0.0f },
                overlapX
            };
        } else {
            return {
                true,
                { 0.0f, (deltaY < 0) ? 1.0f : -1.0f },
                overlapY
            };
        }
    }
    return {false, {0.0f, 0.0f}, 0.0f};
}