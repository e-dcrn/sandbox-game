#pragma once
#include <entt/entt.hpp>
#include <SDL.h>
#include <cmath>
#include "../common/Components.hpp"

class EntityBuilder {
public:
    explicit EntityBuilder(entt::registry& registry) : m_registry(registry) {}


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


    EntityBuilder& at(float x, float y) {
        m_x = x;
        m_y = y;
        return *this;
    }

    EntityBuilder& withAngle(float degrees) {
        m_theta = degrees * (3.14159f / 180.0f);
        return *this;
    }

    EntityBuilder& withVelocity(float vx, float vy) {
        m_vx = vx;
        m_vy = vy;
        return *this;
    }

    EntityBuilder& withAngularVelocity(float degreesPerSecond) {
        m_omega = degreesPerSecond * (3.14159f / 180.0f);
        return *this;
    }


    EntityBuilder& withMass(float mass) {
        m_mass = mass;
        if (!m_customInertiaMassSet) {
            m_inertiaMass = mass;
        }
        return *this;
    }

    EntityBuilder& withInertiaMass(float mass) {
        m_inertiaMass = mass;
        m_customInertiaMassSet = true;
        return *this;
    }

    EntityBuilder& withPhysicsMaterial(float restitution, float static_friction = 1.0f, float dynamic_friction = 1.0f) {
        m_restitution = restitution;
        m_static_friction = static_friction; 
        m_dynamic_friction = dynamic_friction;
        return *this;
    }


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


    entt::entity build() {
        auto entity = m_registry.create();

        m_registry.emplace<Position>(entity, m_x, m_y, m_theta);
        m_registry.emplace<Velocity>(entity, m_vx, m_vy, m_omega);
        
        m_registry.emplace<ForceAccumulator>(entity, 0.0f, 0.0f/*, 0.0f*/);

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

        float inertia = 0.0f;
        
        if (m_inertiaMass > 0.0f) {
            if (m_type == ShapeType::CIRCLE) {
                inertia = 0.5f * m_inertiaMass * m_radius * m_radius;
            } else if (m_type == ShapeType::BOX) {
                inertia = (1.0f / 12.0f) * m_inertiaMass * (m_width * m_width + m_height * m_height);
            }
        }

        float invMass = (m_mass > 0.0f) ? 1.0f / m_mass : 0.0f;
        float invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;

        m_registry.emplace<RigidBody>(entity, m_mass, invMass, m_restitution, inertia, invInertia, m_static_friction, m_dynamic_friction);

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

    enum class ShapeType { NONE, CIRCLE, BOX } m_type = ShapeType::NONE;
    float m_x = 0, m_y = 0;
    float m_vx = 0, m_vy = 0, m_theta = 0;
    float m_radius = 10.0f;
    float m_width = 10.0f, m_height = 10.0f;
    float m_omega = 0.0f;
    
    float m_mass = 0.0f;          
    float m_inertiaMass = 0.0f;   
    bool m_customInertiaMassSet = false;

    float m_restitution = 0.5f;
    float m_static_friction = 0.0f;
    float m_dynamic_friction = 0.0f;
    
    bool m_hasGravity = false;
    bool m_hasCollider = false;
    bool m_hasRaycast = false;

    SDL_Color m_color = {255, 255, 255, 255};
};