#include "PhysicsSystem.hpp"
#include "forces/Gravity.hpp"

constexpr int PHYSICS_SUB_STEPS = 1;
constexpr float MAX_REWIND_TIME = 0.05f;

void PhysicsSystem::init() {
    force_registry.add([](entt::registry& r, float /*dt*/) { 
        Forces::apply_gravity(r, 980.0f); 
    });

    hash_grid.resize(HASH_SIZE);
    for(auto& cell : hash_grid) cell.particle_indices.reserve(16);
    cache.reserve(3000);
    processed_pairs.reserve(6000);
}

void PhysicsSystem::update(entt::registry& registry, float dt) {

    float sub_dt = dt / PHYSICS_SUB_STEPS;

    for(int i = 0; i < PHYSICS_SUB_STEPS; ++i){
        registry.view<ForceAccumulator>().each([](auto& force) {
            force.x = 0; force.y = 0;
        });

        force_registry.apply_all(registry, sub_dt);

        auto view = registry.view<RigidBody, Position, Velocity, Collider, ForceAccumulator>();
        view.each([&registry, sub_dt, this](auto entity, const auto& body, auto& pos, auto& vel, const auto& col, const auto& forces) {
            if (body.inverseMass == 0 && body.inverseInertia == 0 && vel.omega == 0 && vel.x == 0 && vel.y == 0) return;
            
            Vect2 acceleration = Vect2{forces.x, forces.y} * body.inverseMass;
            vel.x += acceleration.x * sub_dt;
            vel.y += acceleration.y * sub_dt;
            pos.theta += vel.omega * sub_dt;
            float dx = vel.x * sub_dt;
            float dy = vel.y * sub_dt;

            // RaycastHit wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            Collision::RaycastHit wall_hit;
            wall_hit.hit = false;
            if (registry.any_of<UseRaycast>(entity)) {
                wall_hit = sweep_static_collisions(registry, pos, col, dx, dy);
            }
            apply_continuous_movement(pos, vel, body, wall_hit, dx, dy, sub_dt);
        });

        sync_from_registry(registry);
        for(int j = 0; j < 6; ++j){
            solve_collisions();
        }
        solve_collisions();
        apply_changes_to_registry(registry);
    }
}

int PhysicsSystem::get_spatial_hash(int cx, int cy) const {
    const uint32_t p1 = 73856093u;
    const uint32_t p2 = 19349663u;
    uint32_t h = (static_cast<uint32_t>(cx) * p1) ^ (static_cast<uint32_t>(cy) * p2);
    return static_cast<int>(h % HASH_SIZE);
}

void PhysicsSystem::sync_from_registry(entt::registry& registry) {
    cache.clear();
    auto view = registry.view<Position, Collider>();
    
    view.each([this, &registry](auto entity, const auto& pos, const auto& col) {
        float vx = 0, vy = 0, omega = 0, invMass = 0, restitution = 1.0f, invInertia = 0.0f, staticFriction = 0.0f, dynamicFriction = 0.0f;

        if (registry.all_of<RigidBody>(entity)) {
            const auto& body = registry.get<RigidBody>(entity);
            invMass = body.inverseMass;
            restitution = body.restitution;
            invInertia = body.inverseInertia;
            staticFriction = body.staticFriction;
            dynamicFriction = body.dynamicFriction;
        }
        if (registry.all_of<Velocity>(entity)) {
            const auto& vel = registry.get<Velocity>(entity);
            vx = vel.x; vy = vel.y; omega = vel.omega;
        }

        int type = (col.type == ColliderType::CIRCLE) ? 0 : 1;
        float hw = col.width * 0.5f;
        float hh = col.height * 0.5f;

        cache.push_back({entity, pos.x, pos.y, pos.theta, vx, vy, omega, vx, vy, omega, col.radius, invMass, invInertia, restitution, staticFriction, dynamicFriction, type, hw, hh});
    });
}

void PhysicsSystem::refresh_grid_from_cache() {
    for(auto& cell : hash_grid) cell.particle_indices.clear();

    for(int i = 0; i < (int)cache.size(); ++i) {
        const auto& p = cache[i];

        Vect2 aabb = PhysicsMath::compute_aabb_half_dims(p.type, p.r, p.hw, p.hh, p.theta);
        auto bounds = PhysicsMath::compute_grid_bounds(p.x, p.y, aabb.x, aabb.y, CELL_SIZE);

        for (int y = bounds.startY; y <= bounds.endY; ++y) {
            for (int x = bounds.startX; x <= bounds.endX; ++x) {
                int hash = get_spatial_hash(x, y);
                hash_grid[hash].particle_indices.push_back(i);
            }
        }
    }
}

void PhysicsSystem::apply_changes_to_registry(entt::registry& registry) {
    for(const auto& p : cache) {
        if (p.invMass > 0 || p.invInertia > 0) {
            auto& pos = registry.get<Position>(p.entity);
            auto& vel = registry.get<Velocity>(p.entity);
            
            pos = { p.x, p.y, p.theta };
            vel = { p.vx, p.vy, p.omega };
        }
    }
}

void PhysicsSystem::solve_sequential_impulse(ParticleCache& pA, ParticleCache& pB, const Collision::Manifold& result) {
    const Vect2 n = result.normal_vector;
    const int count = std::min(result.contact_count, 2);

    Vect2 rA[2], rB[2];
    float vn[2] = {0.0f, 0.0f};
    float target[2] = {0.0f, 0.0f};
    float k[2] = {0.0f, 0.0f};
    float j[2] = {0.0f, 0.0f};

    float e_base = PhysicsMath::combine_restitution(pA.restitution, pB.restitution);

    for (int i = 0; i < count; ++i) {
        Vect2 cp = result.contacts[i];
        rA[i] = cp - Vect2{pA.x, pA.y};
        rB[i] = cp - Vect2{pB.x, pB.y};
        if (pA.type == 0) rA[i] = n * pA.r;
        if (pB.type == 0) rB[i] = -n * pB.r;

        Vect2 velA = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA[i]);
        Vect2 velB = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB[i]);
        vn[i] = PhysicsMath::dot(velB - velA, n);

        Vect2 velA0 = PhysicsMath::get_velocity_at_point({pA.vx0, pA.vy0}, pA.omega0, rA[i]);
        Vect2 velB0 = PhysicsMath::get_velocity_at_point({pB.vx0, pB.vy0}, pB.omega0, rB[i]);
        float vn0 = PhysicsMath::dot(velB0 - velA0, n);

        float e = (std::abs(vn0) < 20.0f) ? 0.0f : e_base;
        target[i] = (vn0 < 0.0f) ? -e * vn0 : 0.0f;

        k[i] = PhysicsMath::calculate_total_impulse_denominator(
            pA.invMass, pA.invInertia, rA[i],
            pB.invMass, pB.invInertia, rB[i], n);
    }

    if (count == 1) {
        if (vn[0] < target[0] && k[0] > 0.0f) j[0] = (target[0] - vn[0]) / k[0];
    } else if (count == 2) {
        float rnA0 = PhysicsMath::cross(rA[0], n), rnA1 = PhysicsMath::cross(rA[1], n);
        float rnB0 = PhysicsMath::cross(rB[0], n), rnB1 = PhysicsMath::cross(rB[1], n);
        float k12 = pA.invMass + pB.invMass + pA.invInertia * rnA0 * rnA1 + pB.invInertia * rnB0 * rnB1;
        float b0 = target[0] - vn[0];
        float b1 = target[1] - vn[1];
        float det = k[0] * k[1] - k12 * k12;
        bool solved = false;

        if (det > 1e-4f * k[0] * k[1]) {
            float x0 = (k[1] * b0 - k12 * b1) / det;
            float x1 = (k[0] * b1 - k12 * b0) / det;
            if (x0 >= 0.0f && x1 >= 0.0f) { j[0] = x0; j[1] = x1; solved = true; }
        }
        if (!solved && k[0] > 0.0f) {
            float x0 = b0 / k[0];
            if (x0 >= 0.0f && vn[1] + k12 * x0 >= target[1]) { j[0] = x0; solved = true; }
        }
        if (!solved && k[1] > 0.0f) {
            float x1 = b1 / k[1];
            if (x1 >= 0.0f && vn[0] + k12 * x1 >= target[0]) { j[1] = x1; solved = true; }
        }
    }

    for (int i = 0; i < count; ++i) {
        if (j[i] <= 0.0f) continue;
        Vect2 impulse = n * j[i];
        pA.vx -= impulse.x * pA.invMass;
        pA.vy -= impulse.y * pA.invMass;
        pA.omega -= PhysicsMath::cross(rA[i], impulse) * pA.invInertia;

        pB.vx += impulse.x * pB.invMass;
        pB.vy += impulse.y * pB.invMass;
        pB.omega += PhysicsMath::cross(rB[i], impulse) * pB.invInertia;
    }

    for (int c = 0; c < count; ++c) {
        if (j[c] <= 0.0f) continue;
        const Vect2& rA_c = rA[c];
        const Vect2& rB_c = rB[c];
        {
            Vect2 velA_f = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA_c);
            Vect2 velB_f = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB_c);
            Vect2 relVel_f = velB_f - velA_f;

            Vect2 tangent = {-result.normal_vector.y, result.normal_vector.x};
            
            float velAlongTangent = PhysicsMath::dot(relVel_f, tangent);
            if (velAlongTangent < 0.0f) {
                tangent = {-tangent.x, -tangent.y};
                velAlongTangent = -velAlongTangent; 
            }

            float denomTangent = PhysicsMath::calculate_total_impulse_denominator(
                pA.invMass, pA.invInertia, rA_c, 
                pB.invMass, pB.invInertia, rB_c, 
                tangent
            );

            if (denomTangent > 0.0f) {
                float jt = -velAlongTangent / denomTangent; 

                float mu_s = std::min(pA.staticFriction, pB.staticFriction);
                float mu_d = std::min(pA.dynamicFriction, pB.dynamicFriction);

                float frictionImpulse;
                if (std::abs(jt) <= j[c] * mu_s) {
                    frictionImpulse = jt;
                } else {
                    frictionImpulse = -j[c] * mu_d;
                }

                Vect2 frictionVec = tangent * frictionImpulse;

                pA.vx -= frictionVec.x * pA.invMass;
                pA.vy -= frictionVec.y * pA.invMass;
                pA.omega -= (rA_c.x * frictionVec.y - rA_c.y * frictionVec.x) * pA.invInertia;

                pB.vx += frictionVec.x * pB.invMass;
                pB.vy += frictionVec.y * pB.invMass;
                pB.omega += (rB_c.x * frictionVec.y - rB_c.y * frictionVec.x) * pB.invInertia;
            }
        }
    }
}

void PhysicsSystem::resolve_contact(ParticleCache& pA, ParticleCache& pB) {
    Collision::Manifold result = {false, {0,0}, 0, 0, {{0,0}, {0,0}}};

    if (pA.type == 0 && pB.type == 0) { 
        result = Collision::test_circles({pA.x, pA.y}, pA.r, {pB.x, pB.y}, pB.r);
    }
    else if (pA.type == 0 && pB.type == 1) { 
        result = Collision::test_circle_box_rotated({pA.x, pA.y}, pA.r, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
        result.normal_vector = -result.normal_vector;
    }
    else if (pA.type == 1 && pB.type == 0) { 
        result = Collision::test_circle_box_rotated({pB.x, pB.y}, pB.r, {pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta);
    }
    else { 
        result = Collision::test_boxes_rotated({pA.x, pA.y}, {pA.hw * 2.0f, pA.hh * 2.0f}, pA.theta, {pB.x, pB.y}, {pB.hw * 2.0f, pB.hh * 2.0f}, pB.theta);
    }

    if (result.is_colliding) {
        float totalMass = pA.invMass + pB.invMass;
        if (totalMass == 0.0f && pA.invInertia == 0.0f && pB.invInertia == 0.0f) return;

        Vect2 mainContact = result.contacts[0];
        Vect2 rA_check = mainContact - Vect2{pA.x, pA.y};
        Vect2 rB_check = mainContact - Vect2{pB.x, pB.y};
        Vect2 vA = PhysicsMath::get_velocity_at_point({pA.vx, pA.vy}, pA.omega, rA_check);
        Vect2 vB = PhysicsMath::get_velocity_at_point({pB.vx, pB.vy}, pB.omega, rB_check);
        float vNormal = PhysicsMath::dot(vB - vA, result.normal_vector);

        const float IMPACT_THRESHOLD = 100.0f;

        if (vNormal < -IMPACT_THRESHOLD) {
            float timeToRewind = result.depth / -vNormal;
            if (timeToRewind > MAX_REWIND_TIME) timeToRewind = MAX_REWIND_TIME; 

            PhysicsMath::integrate_position(pA.x, pA.y, pA.theta, pA.vx, pA.vy, pA.omega, -timeToRewind);
            PhysicsMath::integrate_position(pB.x, pB.y, pB.theta, pB.vx, pB.vy, pB.omega, -timeToRewind);

            solve_sequential_impulse(pA, pB, result);

            PhysicsMath::integrate_position(pA.x, pA.y, pA.theta, pA.vx, pA.vy, pA.omega, timeToRewind);
            PhysicsMath::integrate_position(pB.x, pB.y, pB.theta, pB.vx, pB.vy, pB.omega, timeToRewind);
        } 
        
        else {
            const float SLOP = 0.01f; 
            const float BAUMGARTE = 0.2f / 3.0f;

            float penetration = std::max(0.0f, result.depth - SLOP);
            
            if (penetration > 0.0f) {
                float correctionMag = (penetration / totalMass) * BAUMGARTE;
                
                Vect2 correction = result.normal_vector * correctionMag;

                const float POS_CORRECTION_THRESHOLD = 0.01f;

                if (std::abs(correction.x) < POS_CORRECTION_THRESHOLD) correction.x = 0.0f;
                if (std::abs(correction.y) < POS_CORRECTION_THRESHOLD) correction.y = 0.0f;

                pA.x -= correction.x * pA.invMass; pA.y -= correction.y * pA.invMass;
                pB.x += correction.x * pB.invMass; pB.y += correction.y * pB.invMass;
            }

            solve_sequential_impulse(pA, pB, result);
        }
    }
}

void PhysicsSystem::solve_collisions() {
    refresh_grid_from_cache();

    processed_pairs.clear();

    for(int i = 0; i < (int)cache.size(); ++i) {
        ParticleCache& pA = cache[i];
        
        if (pA.invMass == 0 && pA.invInertia == 0) continue;

        Vect2 aabbA = PhysicsMath::compute_aabb_half_dims(pA.type, pA.r, pA.hw, pA.hh, pA.theta);
        auto bounds = PhysicsMath::compute_grid_bounds(pA.x, pA.y, aabbA.x, aabbA.y, CELL_SIZE);

        for (int gy = bounds.startY; gy <= bounds.endY; ++gy) {
            for (int gx = bounds.startX; gx <= bounds.endX; ++gx) {
                int hash = get_spatial_hash(gx, gy);
                const auto& cell = hash_grid[hash];

                for(int j : cell.particle_indices) {
                    if (i == j) continue;

                    int id1 = i, id2 = j;
                    if (id1 > id2) std::swap(id1, id2);
                    uint64_t pair_key = (static_cast<uint64_t>(id1) << 32) | static_cast<uint32_t>(id2);

                    if (processed_pairs.find(pair_key) != processed_pairs.end()) continue;
                    processed_pairs.insert(pair_key);

                    ParticleCache& pB = cache[j];
                    
                    resolve_contact(pA, pB);
                }
            }
        }
    }
}

Collision::RaycastHit PhysicsSystem::sweep_static_collisions(entt::registry& registry, 
                                              const Position& pos, 
                                              const Collider& body_collider, 
                                              float dx, float dy) {
    Collision::RaycastHit best_hit;
    best_hit.t = 1.0f;

    float body_radius = body_collider.radius; 
    if(body_collider.type == ColliderType::BOX) body_radius = std::min(body_collider.width, body_collider.height) / 2.0f;

    auto statics = registry.view<Position, Collider, RigidBody>();

    statics.each([&](auto wall, const auto& wall_pos, const auto& wall_col, const auto& wall_body) {
        if (wall_body.inverseMass != 0.0f || wall_body.inverseInertia != 0.0f) return;
        if (const auto* wall_vel = registry.try_get<Velocity>(wall)) {
            if (wall_vel->x != 0.0f || wall_vel->y != 0.0f || wall_vel->omega != 0.0f) return;
        }

        Collision::RaycastHit current_hit;

        if (wall_col.type == ColliderType::BOX) {
            float expanded_w = wall_col.width + body_radius * 2.0f;
            float expanded_h = wall_col.height + body_radius * 2.0f;
            
            current_hit = Collision::raycast_box_rotated(pos.x, pos.y, dx, dy, 
                                      wall_pos.x, wall_pos.y, expanded_w, expanded_h, wall_pos.theta);
        } 
        else if (wall_col.type == ColliderType::CIRCLE) {
            float expanded_radius = wall_col.radius + body_radius;
            
            current_hit = Collision::raycast_circle(pos.x, pos.y, dx, dy, 
                                        wall_pos.x, wall_pos.y, expanded_radius);
        }

        if (current_hit.t < 0.0f) return;
        if (current_hit.hit && current_hit.t < best_hit.t) {
            best_hit = current_hit;
        }
    });

    return best_hit;
}

void PhysicsSystem::apply_continuous_movement(Position& pos, Velocity& vel, const RigidBody& body,
                                              const Collision::RaycastHit& hit, float dx, float dy, float dt) {
    if (!hit.hit) {
        pos.x += dx; 
        pos.y += dy;
        return;
    }

    float safe_t = std::max(0.0f, hit.t - 0.001f);
    pos.x += dx * safe_t;
    pos.y += dy * safe_t;
    
    float velAlongNormal = PhysicsMath::dot({vel.x, vel.y}, hit.normal);
    
    if (velAlongNormal < 0) {
        vel.x -= velAlongNormal * hit.normal.x;
        vel.y -= velAlongNormal * hit.normal.y;
    }
}