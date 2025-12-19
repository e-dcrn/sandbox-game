#pragma once
#include <entt/entt.hpp>
#include <SDL.h>
#include <cmath>
#include "../common/Components.hpp"

class EntityBuilder {
public:
    explicit EntityBuilder(entt::registry& registry) : m_registry(registry) {}

    // --- 1. Type et Forme ---

    EntityBuilder& asCircle(float radius) {
        m_type = ShapeType::CIRCLE;
        m_radius = radius;
        return *this;
    }

    EntityBuilder& asBox(float width, float height) {
        m_type = ShapeType::BOX;
        m_width = width;
        m_height = height;
        return *this;
    }

    // --- 2. Position et Vitesse ---

    EntityBuilder& at(float x, float y) {
        m_x = x;
        m_y = y;
        return *this;
    }

    EntityBuilder& withAngle(float degrees) {
        radians = degrees * (3.14159f / 180.0f);
        return *this;
    }

    EntityBuilder& withVelocity(float vx, float vy) {
        m_vx = vx;
        m_vy = vy;
        return *this;
    }

    EntityBuilder& withAngularVelocity(float degreesPerSecond) {
        m_omega = degreesPerSecond * (3.14159f / 180.0f); // Conversion deg -> rad
        return *this;
    }

    // --- 3. Physique (Masse, Inertie, Matériau) ---

    EntityBuilder& withMass(float mass) {
        m_mass = mass;
        // Par défaut, si on n'a pas encore touché à l'inertie manuelle, 
        // la masse d'inertie suit la masse principale.
        if (!m_customInertiaMassSet) {
            m_inertiaMass = mass;
        }
        return *this;
    }

    // Permet de définir une masse différente juste pour le calcul de l'inertie
    // (Ex: Objet fixe mass=0, mais qui peut tourner massInertia=50)
    EntityBuilder& withInertiaMass(float mass) {
        m_inertiaMass = mass;
        m_customInertiaMassSet = true;
        return *this;
    }

    EntityBuilder& withPhysicsMaterial(float restitution, float friction = 0.0f) {
        m_restitution = restitution;
        m_friction = friction; // Sera utilisé quand ajouté au RigidBody
        return *this;
    }

    // --- 4. Comportements et Flags ---

    EntityBuilder& withGravity() {
        m_hasGravity = true;
        return *this;
    }

    EntityBuilder& withCollider() {
        m_hasCollider = true;
        return *this;
    }

    EntityBuilder& withRaycast() {
        m_hasRaycast = true;
        return *this;
    }

    EntityBuilder& withColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
        m_color = {r, g, b, a};
        return *this;
    }

    // --- 5. Construction Finale ---

    entt::entity build() {
        auto entity = m_registry.create();

        // 1. Position & Vélocité
        m_registry.emplace<Position>(entity, m_x, m_y, radians);
        m_registry.emplace<Velocity>(entity, m_vx, m_vy, m_omega);
        
        // 3. Force Accumulator (Indispensable)
        m_registry.emplace<ForceAccumulator>(entity, 0.0f, 0.0f/*, 0.0f*/);

        // 4. Forme et Rendu
        if (m_type == ShapeType::CIRCLE) {
            m_registry.emplace<Circle>(entity, m_radius, m_color);
            if (m_hasCollider) {
                m_registry.emplace<Collider>(entity, ColliderType::CIRCLE, m_radius, 0.0f, 0.0f);
            }
        } 
        else if (m_type == ShapeType::BOX) {
            m_registry.emplace<Rectangle>(entity, m_width, m_height, m_color);
            if (m_hasCollider) {
                m_registry.emplace<Collider>(entity, ColliderType::BOX, 0.0f, m_width, m_height);
            }
        }

        // 5. RigidBody (Calculs automatiques)
        float inertia = 0.0f;
        
        // Calcul de l'inertie basé sur la géométrie et la "Masse d'Inertie"
        if (m_inertiaMass > 0.0f) {
            if (m_type == ShapeType::CIRCLE) {
                inertia = 0.5f * m_inertiaMass * m_radius * m_radius;
            } else if (m_type == ShapeType::BOX) {
                inertia = (1.0f / 12.0f) * m_inertiaMass * (m_width * m_width + m_height * m_height);
            }
        }

        float invMass = (m_mass > 0.0f) ? 1.0f / m_mass : 0.0f;
        float invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;

        // Note: On pourrait ajouter m_friction dans RigidBody plus tard
        m_registry.emplace<RigidBody>(entity, m_mass, invMass, m_restitution, inertia, invInertia);

        // 6. Flags
        if (m_hasGravity) {
            m_registry.emplace<GravityAffected>(entity);
        }
        if (m_hasRaycast) {
            m_registry.emplace<UseRaycast>(entity);
        }

        return entity;
    }

private:
    entt::registry& m_registry;

    // Default values
    enum class ShapeType { NONE, CIRCLE, BOX } m_type = ShapeType::NONE;
    float m_x = 0, m_y = 0;
    float m_vx = 0, m_vy = 0, radians = 0;
    float m_radius = 10.0f;
    float m_width = 10.0f, m_height = 10.0f;
    float m_omega = 0.0f;
    
    float m_mass = 0.0f;          // Par défaut statique
    float m_inertiaMass = 0.0f;   // Par défaut statique en rotation aussi
    bool m_customInertiaMassSet = false;

    float m_restitution = 0.5f;
    float m_friction = 0.0f;
    
    bool m_hasGravity = false;
    bool m_hasCollider = false;
    bool m_hasRaycast = false;

    SDL_Color m_color = {255, 255, 255, 255};
};