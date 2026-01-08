#pragma once

#include <cmath>
#include <algorithm>
#include <array> // Nécessaire pour std::array
#include "../common/Components.hpp"
#include "PhysicsMath.hpp"

namespace Collision {

    // Structure mise à jour pour supporter jusqu'à 2 points de contact
    struct Manifold {
        bool is_colliding;
        Vect2 normal_vector;
        float depth;
        int contact_count;      // Nombre de points (0, 1 ou 2)
        Vect2 contacts[2];      // Tableau des points de contact
    };
    
    struct RaycastHit {
        bool hit = false;
        float t = 1.0f;        // 0.0 à 1.0
        Vect2 normal = {0, 0};
    };
    
    inline Manifold test_circles(Vect2 centerA, float radiusA, Vect2 centerB, float radiusB) {
        Vect2 delta = centerB - centerA;
        float square_dist = PhysicsMath::dot(delta, delta);
        
        float required_distance = radiusA + radiusB;
        float required_square = required_distance * required_distance;
        
        if (square_dist < required_square) {
            float dist = std::sqrt(square_dist);
            Vect2 normal;
            Vect2 contact;
    
            if (dist == 0.0f) {
                normal = {0.0f, 1.0f};
                contact = centerA + Vect2{0.0f, radiusA};
                // Retourne 1 point de contact
                return { true, normal, required_distance, 1, {contact, {0,0}} }; 
            }
            normal = PhysicsMath::normalize(delta);
            contact = centerA + normal * radiusA;
    
            return {
                true,
                normal,
                required_distance - dist,
                1,              // Count
                {contact, {0,0}} // Contacts array
            };
        }
        return {false, {0.0f, 0.0f}, 0.0f, 0, {{0,0}, {0,0}}};
    }
    
    inline Manifold test_circle_box(Vect2 circle_center, float circle_radius, Vect2 box_center, Vect2 box_dimensions) {
        Vect2 delta = circle_center - box_center;
        float halfW = box_dimensions.x * 0.5f;
        float halfH = box_dimensions.y * 0.5f;
    
        float closestX = std::clamp(delta.x, -halfW, halfW);
        float closestY = std::clamp(delta.y, -halfH, halfH);
    
        bool inside = (delta.x == closestX && delta.y == closestY);
    
        if (inside) {
            float distToEdgeX = halfW - std::abs(delta.x);
            float distToEdgeY = halfH - std::abs(delta.y);
    
            Vect2 normal;
            float depth;
            Vect2 contact;
    
            if (distToEdgeX < distToEdgeY) {
                float sign = (delta.x > 0) ? 1.0f : -1.0f;
                normal = { (delta.x > 0) ? 1.0f : -1.0f, 0.0f };
                depth = distToEdgeX + circle_radius;
                contact = box_center + Vect2{ halfW * sign, delta.y };
            } 
            else {
                float sign = (delta.y > 0) ? 1.0f : -1.0f;
                normal = { 0.0f, (delta.y > 0) ? 1.0f : -1.0f };
                depth = distToEdgeY + circle_radius;
                contact = box_center + Vect2{ delta.x, halfH * sign };
            }
    
            return { true, normal, depth, 1, {contact, {0,0}} };
        }
        else {
            Vect2 contact = box_center + Vect2{closestX, closestY};
    
            Vect2 diff = delta - Vect2{closestX, closestY};
            float distSq = PhysicsMath::dot(diff, diff);
    
            if (distSq > circle_radius * circle_radius) {
                return {false, {0.0f, 0.0f}, 0.0f, 0, {{0,0}, {0,0}}};
            }
    
            float dist = std::sqrt(distSq);
            Vect2 normal = PhysicsMath::normalize(diff);
            if (dist == 0.0f) normal = {0.0f, 1.0f};
    
            return { true, normal, circle_radius - dist, 1, {contact, {0,0}} };
        }
    }
    
    inline Manifold test_circle_box_rotated(Vect2 circlePos, float radius, 
                                                    Vect2 boxPos, Vect2 boxDim, float boxAngle) {
        Vect2 relPos = circlePos - boxPos;
        
        // Passage en local
        Vect2 localCirclePos = PhysicsMath::rotate(relPos, -boxAngle);
    
        Manifold localResult = test_circle_box(localCirclePos, radius, {0.0f, 0.0f}, boxDim);
    
        if (localResult.is_colliding) {
            Manifold worldResult;
            worldResult.is_colliding = true;
            worldResult.normal_vector = PhysicsMath::rotate(localResult.normal_vector, boxAngle);
            worldResult.depth = localResult.depth;
            worldResult.contact_count = localResult.contact_count;

            // On rotate tous les points de contact trouvés (généralement 1 pour un cercle)
            for(int i=0; i < localResult.contact_count; ++i) {
                worldResult.contacts[i] = PhysicsMath::rotate(localResult.contacts[i], boxAngle) + boxPos;
            }
            return worldResult;
        }
    
        return { false, {0,0}, 0, 0, {{0,0}, {0,0}} };
    }
    
    inline Manifold test_boxes(Vect2 centerA, Vect2 dimA, Vect2 centerB, Vect2 dimB) {
        float halfW_A = dimA.x * 0.5f;
        float halfH_A = dimA.y * 0.5f;
        float halfW_B = dimB.x * 0.5f;
        float halfH_B = dimB.y * 0.5f;
    
        Vect2 delta = centerB - centerA;
        float overlapX = halfW_A + halfW_B - std::abs(delta.x);
        float overlapY = halfH_A + halfH_B - std::abs(delta.y);
    
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
    
            Vect2 contact = (Vect2{intersectMinX, intersectMinY} + Vect2{intersectMaxX, intersectMaxY}) * 0.5f;
            
            Vect2 normal;
            float depth;

            if (overlapX < overlapY) {
                normal = { (delta.x < 0) ? -1.0f : 1.0f, 0.0f };
                depth = overlapX;
            } else {
                normal = { 0.0f, (delta.y < 0) ? -1.0f : 1.0f };
                depth = overlapY;
            }

            return { true, normal, depth, 1, {contact, {0,0}} };
        }
        return {false, {0.0f, 0.0f}, 0.0f, 0, {{0,0}, {0,0}} };
    }
    
    inline void compute_box_vertices(Vect2 center, Vect2 dim, float angle, std::array<Vect2, 4>& vertices) {
        float hw = dim.x * 0.5f;
        float hh = dim.y * 0.5f;
        Vect2 corners[4] = { {-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh} };
        
        for(int i=0; i<4; i++) {
            vertices[i] = center + PhysicsMath::rotate(corners[i], angle);
        }
    }
    
    inline void project_on_axis(const std::array<Vect2, 4>& vertices, Vect2 axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = std::numeric_limits<float>::lowest();
        for (const auto& v : vertices) {
            float proj = PhysicsMath::dot(v, axis);
            if (proj < min) min = proj;
            if (proj > max) max = proj;
        }
    }
    
   inline Manifold test_boxes_rotated(Vect2 posA, Vect2 dimA, float angleA,
                                    Vect2 posB, Vect2 dimB, float angleB) {
        std::array<Vect2, 4> vertsA{}, vertsB{};
        compute_box_vertices(posA, dimA, angleA, vertsA);
        compute_box_vertices(posB, dimB, angleB, vertsB);

        Vect2 axes[4];
        axes[0] = PhysicsMath::normalize(vertsA[1] - vertsA[0]); 
        axes[1] = PhysicsMath::normalize(vertsA[1] - vertsA[2]);
        axes[2] = PhysicsMath::normalize(vertsB[1] - vertsB[0]);
        axes[3] = PhysicsMath::normalize(vertsB[1] - vertsB[2]); 

        float minOverlap = std::numeric_limits<float>::max();
        Vect2 smallestAxis = {0, 0};
        int bestAxisIndex = -1; 

        // 1. SAT Loop (Séparation Axis Theorem)
        for (int i = 0; i < 4; i++) {
            float minA, maxA, minB, maxB;
            project_on_axis(vertsA, axes[i], minA, maxA);
            project_on_axis(vertsB, axes[i], minB, maxB);

            float overlap1 = maxA - minB;
            float overlap2 = maxB - minA;

            if (overlap1 < 0 || overlap2 < 0) return { false, {0,0}, 0, 0, {{0,0}, {0,0}} };

            float overlap = std::min(overlap1, overlap2);

            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axes[i];
                bestAxisIndex = i; 
            }
        }

        Vect2 dir = posB - posA;
        if (PhysicsMath::dot(smallestAxis, dir) < 0) {
            smallestAxis = -smallestAxis;
        }

        // --- DETECTION ALIGNEMENT (TOLERANCE) ---
        const float PI_2 = 1.57079632679f;
        const float ANGLE_EPSILON = 0.05f; 

        float diffAngle = std::abs(angleA - angleB);
        diffAngle = diffAngle - std::floor(diffAngle / PI_2) * PI_2;

        bool isAligned = (diffAngle < ANGLE_EPSILON || diffAngle > (PI_2 - ANGLE_EPSILON));
        
        // Si aligné, on cherche jusqu'à 2 points avec une tolérance large (1.5px)
        // Sinon, on cherche le point unique le plus profond (0.01px)
        float flatTolerance = isAligned ? 1.5f : 0.01f;

        // --- CALCUL DES POINTS DE CONTACT (MULTI-POINTS) ---
        Manifold result;
        result.is_colliding = true;
        result.normal_vector = smallestAxis;
        result.depth = minOverlap;
        result.contact_count = 0; // On commence à 0

        const std::array<Vect2, 4>* vertsToTest;

        if (bestAxisIndex < 2) {
            // Normale A->B, l'incident est B. On cherche le Min Dot sur B.
            vertsToTest = &vertsB;
            float minProj = std::numeric_limits<float>::max();
            
            // 1. Trouver la profondeur maximale (valeur min)
            for(const auto& v : *vertsToTest) {
                float proj = PhysicsMath::dot(v, smallestAxis);
                if(proj < minProj) minProj = proj;
            }
            // 2. Récupérer TOUS les points proches de cette profondeur (Max 2)
            for(const auto& v : *vertsToTest) {
                float proj = PhysicsMath::dot(v, smallestAxis);
                if (std::abs(proj - minProj) <= flatTolerance) {
                    if (result.contact_count < 2) {
                        result.contacts[result.contact_count] = v;
                        result.contact_count++;
                    }
                }
            }
        } else {
            // Normale A->B, l'incident est A. On cherche le Max Dot sur A.
            vertsToTest = &vertsA;
            float maxProj = std::numeric_limits<float>::lowest();

            // 1. Trouver la profondeur maximale (valeur max)
            for(const auto& v : *vertsToTest) {
                float proj = PhysicsMath::dot(v, smallestAxis);
                if(proj > maxProj) maxProj = proj;
            }
            // 2. Récupérer TOUS les points proches de cette profondeur (Max 2)
            for(const auto& v : *vertsToTest) {
                float proj = PhysicsMath::dot(v, smallestAxis);
                if (std::abs(proj - maxProj) <= flatTolerance) {
                    if (result.contact_count < 2) {
                        result.contacts[result.contact_count] = v;
                        result.contact_count++;
                    }
                }
            }
        }

        return result;
    }
    
    // RaycastHit functions remain unchanged (they are single point by definition)
    inline RaycastHit raycast_box(float ox, float oy, float dx, float dy, 
                              float bx, float by, float bw, float bh) {
        RaycastHit result;
        float inv_dx = (dx != 0.0f) ? 1.0f / dx : std::numeric_limits<float>::max();
        float inv_dy = (dy != 0.0f) ? 1.0f / dy : std::numeric_limits<float>::max();
        Vect2 half_dim = Vect2{bw, bh} * 0.5f;
        Vect2 b_min = Vect2{bx, by} - half_dim;
        Vect2 b_max = Vect2{bx, by} + half_dim;
        float t1 = (b_min.x - ox) * inv_dx;
        float t2 = (b_max.x - ox) * inv_dx;
        float t3 = (b_min.y - oy) * inv_dy;
        float t4 = (b_max.y - oy) * inv_dy;
        float tmin = std::max(std::min(t1, t2), std::min(t3, t4));
        float tmax = std::min(std::max(t1, t2), std::max(t3, t4));
        if (tmax < 0 || tmin > tmax || tmin > 1.0f) return result;
        result.hit = true;
        result.t = tmin; 
        if (std::min(t1, t2) > std::min(t3, t4)) {
            result.normal = {(dx < 0) ? 1.0f : -1.0f, 0.0f};
        } else {
            result.normal = {0.0f, (dy < 0) ? 1.0f : -1.0f};
        }
        return result;
    }
    
    inline RaycastHit raycast_circle(float ox, float oy, float dx, float dy, 
                                float cx, float cy, float radius) {
        RaycastHit result;
        Vect2 f = Vect2{ox, oy} - Vect2{cx, cy};
        Vect2 d = {dx, dy};
        float a = PhysicsMath::dot(d, d);
        float b = 2.0f * PhysicsMath::dot(f, d);
        float c = PhysicsMath::dot(f, f) - (radius * radius);
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return result;
        discriminant = std::sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);
        if (t2 < 0 || t1 > 1.0f) return result;
        result.hit = true;
        result.t = t1;
        Vect2 hit_point = Vect2{ox, oy} + Vect2{dx, dy} * result.t;
        result.normal = PhysicsMath::normalize(hit_point - Vect2{cx, cy});
        return result;
    }

}