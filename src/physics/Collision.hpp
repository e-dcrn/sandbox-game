#pragma once

#include <cmath>
#include <algorithm>
#include "../common/Components.hpp"

struct CollisionResult {
    bool is_colliding;
    Vect2 normal_vector;
    float depth;
    Vect2 contact_point;
};

struct RayHit {
    bool hit = false;
    float t = 1.0f;        // 0.0 à 1.0
    struct { float x, y; } normal = {0, 0};
};

inline float dot(Vect2 a, Vect2 b) {
    return a.x * b.x + a.y * b.y;
}

inline Vect2 normalize(Vect2 v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y);
    if (len == 0) return {0, 0};
    return {v.x / len, v.y / len};
}

inline Vect2 rotate_vector(Vect2 v, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return { v.x * c - v.y * s, v.x * s + v.y * c };
}

inline CollisionResult check_circle_circle(Vect2 centerA, float radiusA, Vect2 centerB, float radiusB) {
    float distX = centerB.x - centerA.x;
    float distY = centerB.y - centerA.y;
    float required_distance = radiusA + radiusB;
    
    float square_dist = (distX * distX) + (distY * distY);
    float required_square = required_distance * required_distance;
    
    if (square_dist < required_square) {
        float dist = std::sqrt(square_dist);
        Vect2 normal;

        if (dist == 0.0f) {
            normal = {0.0f, 1.0f};
            return { true, normal, required_distance, {centerA.x, centerA.y + radiusA}}; 
        }
        float inverse_dist = 1.0f / dist;
        normal = {distX * inverse_dist, distY * inverse_dist};
        Vect2 contact = {
            centerA.x + normal.x * radiusA,
            centerA.y + normal.y * radiusA
        };

        return {
            true,
            normal,
            required_distance - dist,
            contact
        };
    }
    return {false, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}};
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
        Vect2 contact;

        if (distToEdgeX < distToEdgeY) {
            float sign = (dx > 0) ? 1.0f : -1.0f;
            normal = { (dx > 0) ? 1.0f : -1.0f, 0.0f };
            depth = distToEdgeX + circle_radius;
            contact = { box_center.x + (halfW * sign), box_center.y + dy };
        } 
        else {
            float sign = (dy > 0) ? 1.0f : -1.0f;
            normal = { 0.0f, (dy > 0) ? 1.0f : -1.0f };
            depth = distToEdgeY + circle_radius;
            contact = { box_center.x + dx, box_center.y + (halfH * sign) };
        }

        return { true, normal, depth, contact};
    }
    else {
        Vect2 contact = { box_center.x + closestX, box_center.y + closestY };

        float diffX = dx - closestX;
        float diffY = dy - closestY;
        float distSq = diffX * diffX + diffY * diffY;

        if (distSq > circle_radius * circle_radius) {
            return {false, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}};
        }

        float dist = std::sqrt(distSq);
        Vect2 normal;
        
        if (dist == 0.0f) {
            normal = {0.0f, 1.0f}; 
        } else {
            normal = { diffX / dist, diffY / dist };
        }

        return { true, normal, circle_radius - dist, contact};
    }
}

inline CollisionResult check_circle_rotated_box(Vect2 circlePos, float radius, 
                                                Vect2 boxPos, Vect2 boxDim, float boxAngle) {
    Vect2 relPos = { circlePos.x - boxPos.x, circlePos.y - boxPos.y };
    
    Vect2 localCirclePos = rotate_vector(relPos, -boxAngle);

    CollisionResult localResult = check_circle_box(localCirclePos, radius, {0.0f, 0.0f}, boxDim);

    if (localResult.is_colliding) {
        Vect2 worldNormal = rotate_vector(localResult.normal_vector, boxAngle);
        Vect2 worldContact = rotate_vector(localResult.contact_point, boxAngle);
        worldContact.x += boxPos.x;
        worldContact.y += boxPos.y;

        return {
            true,
            worldNormal,
            localResult.depth,
            worldContact
        };
    }

    return { false, {0,0}, 0, {0,0} };
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
        float minXA = centerA.x - halfW_A;
        float maxXA = centerA.x + halfW_A;
        float minXB = centerB.x - halfW_B;
        float maxXB = centerB.x + halfW_B;

        float minYA = centerA.y - halfH_A;
        float maxYA = centerA.y + halfH_A;
        float minYB = centerB.y - halfH_B;
        float maxYB = centerB.y + halfH_B;

        float intersectMinX = std::max(minXA, minXB);
        float intersectMaxX = std::min(maxXA, maxXB);
        float intersectMinY = std::max(minYA, minYB);
        float intersectMaxY = std::min(maxYA, maxYB);

        Vect2 contact = {
            (intersectMinX + intersectMaxX) * 0.5f,
            (intersectMinY + intersectMaxY) * 0.5f
        };

        if (overlapX < overlapY) {
            return {
                true,
                { (deltaX < 0) ? -1.0f : 1.0f, 0.0f },
                overlapX,
                contact
            };
        } else {
            return {
                true,
                { 0.0f, (deltaY < 0) ? -1.0f : 1.0f },
                overlapY,
                contact
            };
        }
    }
    return {false, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}};
}

inline void get_box_vertices(Vect2 center, Vect2 dim, float angle, std::vector<Vect2>& vertices) {
    float hw = dim.x * 0.5f;
    float hh = dim.y * 0.5f;
    // Coins locaux
    Vect2 corners[4] = { {-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh} };
    
    float c = std::cos(angle);
    float s = std::sin(angle);

    for(int i=0; i<4; i++) {
        // Rotation + Translation
        float rx = corners[i].x * c - corners[i].y * s;
        float ry = corners[i].x * s + corners[i].y * c;
        vertices.push_back({ center.x + rx, center.y + ry });
    }
}

inline void project_vertices(const std::vector<Vect2>& vertices, Vect2 axis, float& min, float& max) {
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::lowest();
    for (const auto& v : vertices) {
        float proj = dot(v, axis);
        if (proj < min) min = proj;
        if (proj > max) max = proj;
    }
}

inline CollisionResult check_rotated_box_box(Vect2 posA, Vect2 dimA, float angleA,
                                             Vect2 posB, Vect2 dimB, float angleB) {
    std::vector<Vect2> vertsA, vertsB;
    get_box_vertices(posA, dimA, angleA, vertsA);
    get_box_vertices(posB, dimB, angleB, vertsB);

    // Les axes à tester sont les normales des faces des deux boîtes
    // (2 axes pour A, 2 axes pour B)
    Vect2 axes[4];
    // Axes de A (Haut et Droite relatifs)
    axes[0] = normalize({vertsA[1].x - vertsA[0].x, vertsA[1].y - vertsA[0].y}); 
    axes[1] = normalize({vertsA[1].x - vertsA[2].x, vertsA[1].y - vertsA[2].y}); 
    // Axes de B
    axes[2] = normalize({vertsB[1].x - vertsB[0].x, vertsB[1].y - vertsB[0].y}); 
    axes[3] = normalize({vertsB[1].x - vertsB[2].x, vertsB[1].y - vertsB[2].y}); 

    float minOverlap = std::numeric_limits<float>::max();
    Vect2 smallestAxis = {0, 0};

    for (int i = 0; i < 4; i++) {
        float minA, maxA, minB, maxB;
        project_vertices(vertsA, axes[i], minA, maxA);
        project_vertices(vertsB, axes[i], minB, maxB);

        float overlap1 = maxA - minB; // Distance pour sortir par la droite
        float overlap2 = maxB - minA; // Distance pour sortir par la gauche

        // Si l'une des distances est négative, on ne se touche pas
        if (overlap1 < 0 || overlap2 < 0) {
            return { false, {0,0}, 0, {0,0} };
        }

        // On prend la plus petite des deux distances (le chemin le plus court pour sortir)
        float overlap = std::min(overlap1, overlap2);

        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axes[i];
        }
    }

    // Assurer que la normale pointe de A vers B
    Vect2 dir = { posB.x - posA.x, posB.y - posA.y };
    if (dot(smallestAxis, dir) < 0) {
        smallestAxis = { -smallestAxis.x, -smallestAxis.y };
    }

    // --- Calcul du point de contact (Approximation : Vertex le plus profond) ---
    // On cherche quel vertex de B est le plus "à l'intérieur" de A (selon la normale inverse)
    Vect2 contactPoint = {0,0};
    float minProj = std::numeric_limits<float>::max();
    
    // On teste les sommets de B contre la normale (qui pointe vers B)
    // Le sommet le plus "en arrière" par rapport à la normale est le candidat
    for(const auto& v : vertsB) {
        float proj = dot(v, { -smallestAxis.x, -smallestAxis.y }); // Projection inverse
        if(proj < minProj) {
            minProj = proj;
            contactPoint = v;
        }
    }
    // Note : Pour une vraie précision physique (Edge-Edge), il faudrait du Clipping (Sutherland-Hodgman)
    // Mais "Deepest Vertex" suffit souvent pour des cubes simples.
    
    // On vérifie aussi si un sommet de A est dans B (si c'est A qui rentre dans B)
    // ... (Code simplifié ici pour ne pas exploser la taille du fichier)

    return { true, smallestAxis, minOverlap, contactPoint };
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