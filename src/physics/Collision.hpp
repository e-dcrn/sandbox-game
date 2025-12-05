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

CollisionResult check_circle_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions){
    float halfW = box_dimensions.x * 0.5f;
    float halfH = box_dimensions.y * 0.5f;
    Vect2 closest = {
        std::clamp(circle_center.x, box_center.x - halfW, box_center.x + halfW),
        std::clamp(circle_center.y, box_center.y - halfH, box_center.y + halfH)
    };

    float distX = circle_center.x - closest.x;
    float distY = circle_center.y - closest.y;
    float square_dist = (distX * distX) + (distY * distY);
    float required_square = circle_radius * circle_radius;

    if (square_dist < required_square) {
        float dist = std::sqrt(square_dist);
        if (dist == 0.0f) {
            return { true, {0.0f, 1.0f}, circle_radius}; 
        }
        float inverse_dist = 1.0f / dist;

        return {
            true,
            {distX * inverse_dist, distY * inverse_dist},
            circle_radius - dist 
        };
    }
    return {false, {0.0f, 0.0f}, 0.0f};
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