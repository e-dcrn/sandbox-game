#pragma once

#include <cmath>
#include <algorithm>
#include <array>
#include "../common/Components.hpp"
#include "PhysicsMath.hpp"

namespace Collision {

    struct Manifold {
        bool is_colliding;
        Vect2 normal_vector;
        float depth;
        int contact_count;
        Vect2 contacts[2];
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
                return { true, normal, required_distance, 1, {contact, {0,0}} }; 
            }
            normal = PhysicsMath::normalize(delta);
            contact = centerA + normal * radiusA;
    
            return {
                true,
                normal,
                required_distance - dist,
                1,
                {contact, {0,0}}
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
        
        Vect2 localCirclePos = PhysicsMath::rotate(relPos, -boxAngle);
    
        Manifold localResult = test_circle_box(localCirclePos, radius, {0.0f, 0.0f}, boxDim);
    
        if (localResult.is_colliding) {
            Manifold worldResult;
            worldResult.is_colliding = true;
            worldResult.normal_vector = PhysicsMath::rotate(localResult.normal_vector, boxAngle);
            worldResult.depth = localResult.depth;
            worldResult.contact_count = localResult.contact_count;

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

        Manifold result;
        result.is_colliding = true;
        result.normal_vector = smallestAxis;
        result.depth = minOverlap;
        result.contact_count = 0;

        const bool refIsA = (bestAxisIndex < 2);
        const std::array<Vect2, 4>& refVerts = refIsA ? vertsA : vertsB;
        const std::array<Vect2, 4>& incVerts = refIsA ? vertsB : vertsA;
        Vect2 refNormal = refIsA ? smallestAxis : -smallestAxis;

        auto edge_normal = [](const std::array<Vect2, 4>& v, int i) {
            Vect2 e = v[(i + 1) % 4] - v[i];
            return PhysicsMath::normalize(Vect2{e.y, -e.x});
        };

        int refEdge = 0, incEdge = 0;
        float bestRef = std::numeric_limits<float>::lowest();
        float bestInc = std::numeric_limits<float>::max();
        for (int i = 0; i < 4; ++i) {
            float dr = PhysicsMath::dot(edge_normal(refVerts, i), refNormal);
            if (dr > bestRef) { bestRef = dr; refEdge = i; }
            float di = PhysicsMath::dot(edge_normal(incVerts, i), refNormal);
            if (di < bestInc) { bestInc = di; incEdge = i; }
        }

        Vect2 r1 = refVerts[refEdge];
        Vect2 r2 = refVerts[(refEdge + 1) % 4];
        Vect2 tangent = PhysicsMath::normalize(r2 - r1);

        Vect2 clip[2] = { incVerts[incEdge], incVerts[(incEdge + 1) % 4] };

        auto clip_segment = [](Vect2 (&pts)[2], Vect2 n, float offset) {
            float d0 = PhysicsMath::dot(pts[0], n) - offset;
            float d1 = PhysicsMath::dot(pts[1], n) - offset;
            if (d0 > 0.0f && d1 > 0.0f) return false;
            if (d0 * d1 < 0.0f) {
                Vect2 inter = pts[0] + (pts[1] - pts[0]) * (d0 / (d0 - d1));
                if (d0 > 0.0f) pts[0] = inter; else pts[1] = inter;
            }
            return true;
        };

        if (!clip_segment(clip, -tangent, -PhysicsMath::dot(r1, tangent)) ||
            !clip_segment(clip,  tangent,  PhysicsMath::dot(r2, tangent))) {
            result.contacts[0] = clip[0];
            result.contact_count = 1;
            return result;
        }

        float refOffset = PhysicsMath::dot(r1, refNormal);
        for (const Vect2& p : clip) {
            float separation = PhysicsMath::dot(p, refNormal) - refOffset;
            if (separation <= 0.0f && result.contact_count < 2) {
                result.contacts[result.contact_count++] = p;
            }
        }
        if (result.contact_count == 0) {
            result.contacts[0] = clip[0];
            result.contact_count = 1;
        }

        return result;
    }
    
    inline RaycastHit raycast_box_rotated(float ox, float oy, float dx, float dy, 
                                          float bx, float by, float bw, float bh, float angle) {
        RaycastHit result;
        Vect2 rel_origin = {ox - bx, oy - by};
        Vect2 local_o = PhysicsMath::rotate(rel_origin, -angle);
        Vect2 local_d = PhysicsMath::rotate({dx, dy}, -angle);

        float inv_dx = (local_d.x != 0.0f) ? 1.0f / local_d.x : std::numeric_limits<float>::max();
        float inv_dy = (local_d.y != 0.0f) ? 1.0f / local_d.y : std::numeric_limits<float>::max();
        
        Vect2 half_dim = Vect2{bw, bh} * 0.5f;
        Vect2 b_min = -half_dim;
        Vect2 b_max = half_dim;

        float t1 = (b_min.x - local_o.x) * inv_dx;
        float t2 = (b_max.x - local_o.x) * inv_dx;
        float t3 = (b_min.y - local_o.y) * inv_dy;
        float t4 = (b_max.y - local_o.y) * inv_dy;

        float tmin = std::max(std::min(t1, t2), std::min(t3, t4));
        float tmax = std::min(std::max(t1, t2), std::max(t3, t4));
        if (tmax < 0 || tmin > tmax || tmin > 1.0f) return result;

        result.hit = true;
        result.t = tmin; 
        Vect2 local_normal;
        if (std::min(t1, t2) > std::min(t3, t4)) {
            local_normal = {(local_d.x < 0) ? 1.0f : -1.0f, 0.0f};
        } else {
            local_normal = {0.0f, (local_d.y < 0) ? 1.0f : -1.0f};
        }
        result.normal = PhysicsMath::rotate(local_normal, angle);
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