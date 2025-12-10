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
    Vect2 contact_point;
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
            return { true, {0.0f, 1.0f}, required_distance,centerA}; 
        }
        float inverse_dist = 1.0f / dist;

        Vect2 contact;
        contact.x = centerA.x + (distX / dist) * radiusA; 
        contact.y = centerA.y + (distY / dist) * radiusA; 
        
        return {
            true,
            {distX * inverse_dist, distY * inverse_dist},
            required_distance - dist,
            contact
        };
    }
    return {false, {0.0f, 0.0f}, 0.0f,{0.0f, 0.0f}};
}

inline CollisionResult check_circle_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions) {
    float dx = circle_center.x - box_center.x;
    float dy = circle_center.y - box_center.y;
    float halfW = box_dimensions.x * 0.5f;
    float halfH = box_dimensions.y * 0.5f;

    // Point le plus proche (Clamped)
    float closestX = std::clamp(dx, -halfW, halfW);
    float closestY = std::clamp(dy, -halfH, halfH);

    bool inside = (dx == closestX && dy == closestY);

    if (inside) {
        // On est DANS la boite. Il faut trouver le bord le plus proche pour pousser dehors.
        float distToEdgeX = halfW - std::abs(dx);
        float distToEdgeY = halfH - std::abs(dy);

        Vect2 normal;
        float depth;
        Vect2 contact; // <--- CORRECTION IMPORTANTE ICI

        if (distToEdgeX < distToEdgeY) {
            // Plus proche du bord vertical (Gauche/Droite)
            float sign = (dx > 0) ? 1.0f : -1.0f;
            normal = { sign, 0.0f };
            depth = distToEdgeX + circle_radius;
            
            // Le point de contact est SUR LE BORD, pas au centre du cercle
            contact.x = box_center.x + (halfW * sign);
            contact.y = box_center.y + closestY;
        } 
        else {
            // Plus proche du bord horizontal (Haut/Bas)
            float sign = (dy > 0) ? 1.0f : -1.0f;
            normal = { 0.0f, sign };
            depth = distToEdgeY + circle_radius;

            // Le point de contact est SUR LE BORD
            contact.x = box_center.x + closestX;
            contact.y = box_center.y + (halfH * sign);
        }

        return { true, normal, depth, contact };
    }
    else {
        // Cas classique (Dehors)
        Vect2 contact = {
            box_center.x + closestX,
            box_center.y + closestY
        };

        float diffX = dx - closestX;
        float diffY = dy - closestY;
        float distSq = diffX * diffX + diffY * diffY;

        if (distSq > circle_radius * circle_radius) {
            return {false, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}};
        }

        float dist = std::sqrt(distSq);
        Vect2 normal = (dist == 0.0f) ? Vect2{0.0f, 1.0f} : Vect2{diffX / dist, diffY / dist};

        return { true, normal, circle_radius - dist, contact };
    }
}

inline CollisionResult check_circle_rotated_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions, float box_angle) {
    
    // --- ETAPE 1 : Transformation World -> Local ---
    
    // Vecteur du centre de la boite vers le cercle
    float dx = circle_center.x - box_center.x;
    float dy = circle_center.y - box_center.y;

    // Rotation inverse (-angle) pour aligner le cercle avec la boite (qui devient virtuelle en 0,0)
    // Formule rotation : x' = x cos - y sin, y' = x sin + y cos
    float c_inv = std::cos(-box_angle);
    float s_inv = std::sin(-box_angle);

    Vect2 localCirclePos;
    localCirclePos.x = dx * c_inv - dy * s_inv;
    localCirclePos.y = dx * s_inv + dy * c_inv;

    // --- ETAPE 2 : Appel de la fonction AABB ---
    // On triche : on dit que la boite est en {0,0}.
    // Du coup, le 'contact' retourné par cette fonction sera purement local (relatif au centre).
    CollisionResult res = check_circle_box(localCirclePos, circle_radius, {0.0f, 0.0f}, box_dimensions);

    if (!res.is_colliding) {
        // On retourne quand même un contact transformé au cas où, ou juste le défaut
        return {false, {0,0}, 0.0f, {0,0}};
    }

    // --- ETAPE 3 : Transformation Local -> World ---
    
    float c = std::cos(box_angle);
    float s = std::sin(box_angle);

    // A. Rotation de la Normale
    Vect2 worldNormal;
    worldNormal.x = res.normal_vector.x * c - res.normal_vector.y * s;
    worldNormal.y = res.normal_vector.x * s + res.normal_vector.y * c;

    // B. Rotation du Point de Contact
    // res.contact_point est local (car on a passé box_center={0,0} à l'étape 2)
    float contactX = res.contact_point.x * c - res.contact_point.y * s;
    float contactY = res.contact_point.x * s + res.contact_point.y * c;

    // C. Translation finale (On rajoute la position réelle de la boite)
    Vect2 worldContact = {
        box_center.x + contactX,
        box_center.y + contactY
    };

    return { true, worldNormal, res.depth, worldContact };
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