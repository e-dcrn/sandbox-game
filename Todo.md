# TP : Transition vers la Physique Newtonienne (Modulaire)

**Objectif :** Remplacer le mouvement direct par une simulation physique réelle ($F = m \cdot a$), où chaque force est "activable" par entité via des composants spécifiques.

## 1. Création des Données (Components)
- [ ] Créer ou modifier `src/physics/Components.hpp`.
- [ ] Ajouter les structures `RigidBody`, `ForceAccumulator` et le tag `GravityAffected`.

```cpp
// src/physics/Components.hpp
#pragma once

// --- DONNÉES PHYSIQUES DE BASE ---
struct RigidBody {
    float mass;          // Masse en kg
    float inverseMass;   // 1 / mass (0 = infini/statique)
    float restitution;   // Coeff de rebond (0.0 à 1.0)
};

struct ForceAccumulator {
    float x;
    float y;
};

// --- TAGS DE FORCES (La "Liste" des forces actives) ---

// Si une entité a ce composant, la gravité s'applique
struct GravityAffected { 
    float scale = 1.0f; // 1.0 = normal, 0.5 = gravité faible, -1.0 = antigravité
};

// (Futur) Si une entité a ce composant, elle subit la résistance de l'air
struct DragAffected {
    float k = 0.5f; 
};
```

## 2. Implémentation de la Gravité (Filtrée)
- [ ] Créer le dossier `src/physics/forces/`.
- [ ] Créer le fichier `src/physics/forces/Gravity.hpp`.
- [ ] Implémenter la fonction pour qu'elle ne cible **que** les entités avec le tag `GravityAffected`.

```cpp
// src/physics/forces/Gravity.hpp
#pragma once
#include <entt/entt.hpp>
#include "../Components.hpp"

namespace Forces {
    // La gravité est globale (ex: 980), mais modulable par entité via le composant
    inline void apply_gravity(entt::registry& registry, float global_g = 980.0f) { 
        
        // On demande : Les entités qui ont un Corps, un Accumulateur ET le tag Gravité
        auto view = registry.view<const RigidBody, ForceAccumulator, const GravityAffected>();
        
        view.each([global_g](const RigidBody& body, ForceAccumulator& forces, const GravityAffected& grav) {
            // Sécurité : On ignore les murs (masse infinie)
            if (body.inverseMass > 0) {
                // F = m * g * scale
                // On AJOUTE à l'accumulateur
                forces.y += (1.0f / body.inverseMass) * global_g * grav.scale; 
            }
        });
    }
}
```

## 3. Mise à jour du Système Physique
- [ ] Modifier `src/physics/PhysicsSystem.hpp` pour accepter `dt`.
- [ ] Modifier `src/physics/PhysicsSystem.cpp` pour resetter les forces, appliquer la gravité, et intégrer le mouvement.

```cpp
// src/physics/PhysicsSystem.cpp
#include "PhysicsSystem.hpp"
#include "../common/Components.hpp" // Position, Velocity
#include "Components.hpp"           // RigidBody, ForceAccumulator
#include "forces/Gravity.hpp"

void update_physics(entt::registry& registry, float dt) {
    
    // 1. RESET DES FORCES (Crucial : sinon les forces s'accumulent à l'infini)
    registry.view<ForceAccumulator>().each([](auto& force) {
        force.x = 0; force.y = 0;
    });

    // 2. APPLICATION DES FORCES
    // On appelle les fonctions "globales", elles se débrouillent pour trouver leurs entités
    Forces::apply_gravity(registry, 500.0f); 

    // 3. INTÉGRATION NEWTONIENNE (Euler Semi-Implicite)
    auto view = registry.view<RigidBody, Position, Velocity, ForceAccumulator>();
    view.each([dt](const auto& body, auto& pos, auto& vel, const auto& forces) {
        if (body.inverseMass == 0) return; // Objet statique

        // a = F / m
        float acc_x = forces.x * body.inverseMass;
        float acc_y = forces.y * body.inverseMass;

        // v += a * dt
        vel.x += acc_x * dt;
        vel.y += acc_y * dt;

        // p += v * dt
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;

        // Hack temporaire : Collision Sol à 700px
        if (pos.y > 700) {
            pos.y = 700;
            vel.y = -vel.y * 0.6f; // Rebond amorti
        }
    });
}
```

## 4. Composition de l'Entité (Game.cpp)
- [ ] Dans `Game.hpp`, ajouter `Uint64 last_tick = 0;`.
- [ ] Dans `Game::init()`, ajouter les composants physiques **et le tag GravityAffected** à l'entité.

```cpp
// Dans Game::init()
last_tick = SDL_GetTicks64();

auto entity = registry.create();
// ... Position, Velocity, Circle ...

// Physique de base
registry.emplace<RigidBody>(entity, 1.0f, 1.0f, 0.5f);
registry.emplace<ForceAccumulator>(entity, 0.0f, 0.0f);

// Activation des forces : "Je veux que cet objet subisse la gravité"
registry.emplace<GravityAffected>(entity); 
```

- [ ] Dans `Game::update()`, calculer le Delta Time (`dt`) et appeler le système.

```cpp
// Dans Game::update()
Uint64 current_tick = SDL_GetTicks64();
float dt = (current_tick - last_tick) / 1000.0f;
last_tick = current_tick;

// Clamp dt pour éviter les bugs si la fenêtre est déplacée/redimensionnée
if (dt > 0.1f) dt = 0.1f;

update_physics(registry, dt);
```

## Vérification finale
- [ ] Le code compile.
- [ ] L'objet tombe et rebondit.
- [ ] **Test Tags :** Si on commente la ligne `registry.emplace<GravityAffected>(entity)`, l'objet doit flotter dans les airs (ne plus tomber), même s'il a une masse.