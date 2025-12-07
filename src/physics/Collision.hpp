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

struct RayHit {
    bool hit = false;
    float t = 1.0f;        // 0.0 à 1.0
    struct { float x, y; } normal = {0, 0};
};

inline CollisionResult check_circle_circle(Vect2 centerA, float radiusA, Vect2 centerB, float radiusB) {
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

inline CollisionResult check_circle_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions) {
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

inline CollisionResult check_box_box(Vect2 centerA, Vect2 dimA, Vect2 centerB, Vect2 dimB) {
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

inline RayHit ray_vs_aabb(float ox, float oy, float dx, float dy, 
                          float bx, float by, float bw, float bh) {
    RayHit result;
    
    // Evite la division par zéro avec une valeur très grande
    float inv_dx = (dx != 0.0f) ? 1.0f / dx : std::numeric_limits<float>::max();
    float inv_dy = (dy != 0.0f) ? 1.0f / dy : std::numeric_limits<float>::max();

    // Calcul des bornes de la boîte (centrée sur bx, by)
    float min_x = bx - bw * 0.5f;
    float max_x = bx + bw * 0.5f;
    float min_y = by - bh * 0.5f;
    float max_y = by + bh * 0.5f;

    float t1 = (min_x - ox) * inv_dx;
    float t2 = (max_x - ox) * inv_dx;
    float t3 = (min_y - oy) * inv_dy;
    float t4 = (max_y - oy) * inv_dy;

    float tmin = std::max(std::min(t1, t2), std::min(t3, t4));
    float tmax = std::min(std::max(t1, t2), std::max(t3, t4));

    // Pas de collision ou collision derrière
    if (tmax < 0 || tmin > tmax || tmin > 1.0f) return result;

    result.hit = true;
    result.t = tmin; // Clamp à 0 si on démarre dedans

    // Normale
    if (std::min(t1, t2) > std::min(t3, t4)) {
        result.normal = {(dx < 0) ? 1.0f : -1.0f, 0.0f};
    } else {
        result.normal = {0.0f, (dy < 0) ? 1.0f : -1.0f};
    }

    return result;
}

inline RayHit ray_vs_circle(float ox, float oy, float dx, float dy, 
                            float cx, float cy, float radius) {
    RayHit result;

    // Vecteur du Rayon vers le Centre du Cercle (f = origin - center)
    float fx = ox - cx;
    float fy = oy - cy;

    // Equation quadratique: a*t^2 + b*t + c = 0
    // a = D.D (Longueur du vecteur vitesse au carré)
    float a = dx * dx + dy * dy;
    
    // b = 2 * (F . D)
    float b = 2.0f * (fx * dx + fy * dy);
    
    // c = (F . F) - r^2
    float c = (fx * fx + fy * fy) - (radius * radius);

    float discriminant = b * b - 4 * a * c;

    // Pas de solution réelle = le rayon rate le cercle
    if (discriminant < 0) return result;

    // On cherche la plus petite racine positive (le premier point d'impact)
    discriminant = std::sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);

    // On prend le plus petit t positif
    if (t2 < 0 || t1 > 1.0f) return result;

    result.hit = true;
    result.t = t1;

    // Calcul de la normale (Point d'impact - Centre Cercle, normalisé)
    float hit_x = ox + dx * result.t;
    float hit_y = oy + dy * result.t;
    
    float nx = hit_x - cx;
    float ny = hit_y - cy;
    // Normalisation
    float len = std::sqrt(nx*nx + ny*ny);
    if (len > 0) {
        result.normal.x = nx / len;
        result.normal.y = ny / len;
    }
    

    return result;
}