# Roadmap: Projet Sandbox 2D Isométrique (C++ Custom Framework)

Ce document sert de guide de développement pour la création d'un jeu type "Sandbox" en vue isométrique, écrit en C++ sans moteur de jeu complet (Unity/Unreal), mais en utilisant des bibliothèques bas niveau pour gérer le matériel.

---

## 🛠️ Phase 1 : La Stack Technique (Les Outils)

Avant de coder, il faut choisir les "gestionnaires" pour ne pas écrire des drivers GPU à la main.

**Les choix recommandés (Standard de l'industrie pour du C++ from scratch) :**

1.  **Gestion Fenêtre/Input/Graphismes 2D :**
    * *Option A (C++ friendly) :* **SFML 3.x** (Simple, orienté objet, très propre).
    * *Option B (C pur, très performant) :* **SDL2** ou **SDL3**.
2.  **Système d'Entités (ECS) :**
    * **EnTT** : La référence absolue en C++. Indispensable pour une sandbox avec beaucoup d'objets. Ne fais pas ta propre gestion d'objets avec de l'héritage complexe (`class Arbre : public Entity`), tu vas le regretter. Utilise l'ECS.
3.  **Interface UI (Debug) :**
    * **Dear ImGui** : Pour créer des menus de debug, voir les variables en temps réel, éditer le terrain.
4.  **Maths & Utilitaires :**
    * **GLM** (si tu fais de l'OpenGL pur plus tard) ou simplement les vecteurs de SFML/SDL.
    * **nlohmann/json** : Pour sauvegarder/charger tes maps.

---

## 🧱 Phase 2 : Architecture de Base

L'objectif est d'avoir une fenêtre qui s'ouvre et une boucle de jeu propre.

### 2.1 La Game Loop
Ne pas utiliser `while(true) { update(); draw(); }` bêtement. Il faut un **TimeStep fixe** pour que la physique soit stable peu importe la puissance du PC.

* [ ] Créer la classe `Game`.
* [ ] Implémenter le pattern "Fixed Update" (Accumulator pattern).
    * `processInput()`
    * `update(deltaTime)` (Logique jeu)
    * `render()` (Dessin)

### 2.2 Gestionnaire de Ressources (Resource Manager)
Pour ne pas charger la texture "herbe.png" 500 fois si tu as 500 tuiles d'herbe.

* [ ] Créer une classe singleton ou statique `ResourceManager`.
* [ ] Stocker les textures dans une `std::map<std::string, Texture>`.
* [ ] Charger une texture une seule fois et renvoyer un pointeur/référence.

---

## 📐 Phase 3 : Le Monde Isométrique (Le cœur du sujet)

C'est ici que la magie opère. Il faut distinguer l'espace "Grille" (2D tableau) de l'espace "Écran" (Isométrique).



### 3.1 Mathématiques Isométriques
Formules vitales à implémenter dans une classe `IsoMath`.

**Cartésien (Grille) vers Isométrique (Écran) :**
```cpp
// TILE_WIDTH et TILE_HEIGHT sont souvent ratio 2:1 (ex: 64x32)
screen.x = (grid.x - grid.y) * (TILE_WIDTH / 2);
screen.y = (grid.x + grid.y) * (TILE_HEIGHT / 2);
```

## 🎨 Phase 4 : Rendu et Profondeur (Z-Ordering)

En isométrique 2D, la notion de "profondeur" est une illusion d'optique. L'ordre dans lequel on dessine les objets est crucial : ce qui est "plus bas" à l'écran doit être dessiné "après" ce qui est plus haut pour le recouvrir.

### 4.1 L'Algorithme du Peintre (Painter's Algorithm)
On ne peut pas juste faire une boucle `for y; for x`. Si tu as des entités mobiles, il faut tout trier à chaque frame.
* [ ] Créer une liste de `Renderable` (pointeurs vers Tuiles + Entités).
* [ ] **Trier** cette liste avant le rendu.
    * *Formule de tri simple :* `(y * width + x)` pour les tuiles, ou simplement la coordonnée `screen.Y` pour les entités libres.
    * *Règle :* L'objet avec le `Y` d'écran le plus bas (le plus "proche" du joueur) est dessiné en dernier.

### 4.2 La Caméra (View)
Dans une sandbox, le monde est plus grand que la fenêtre.
* [ ] Créer une classe `Camera` qui stocke un `Offset (x, y)`.
* [ ] Lors du rendu, soustraire cet offset à la position de chaque sprite : `sprite.setPosition(isoPos - cameraOffset)`.
* [ ] **Culling (Optimisation)** : Avant de dessiner, vérifier si l'objet est dans le rectangle de la caméra. Si non -> `continue`. C'est vital pour les performances si ta map fait 1000x1000 tuiles.

---

## 🏃 Phase 5 : Entités et Interactions (ECS)

Pour une sandbox, l'Orienté Objet classique (`class Arbre`, `class Joueur`, `class Zombie`) devient vite un enfer à maintenir (le problème du "Diamant"). Utilise un **ECS (Entity Component System)**.

### 5.1 Intégration ECS (Recommandation : EnTT)
* [ ] Installer la librairie **EnTT**.
* [ ] Créer des **Composants** (Structs de données pures, pas de méthodes) :
    * `PositionComponent { float x, y; }` (Position Grille)
    * `SpriteComponent { Texture* tex; IntRect rect; }`
    * `MovableComponent { float speed; Vector2 velocity; }`
* [ ] Créer des **Systèmes** (Logique pure) :
    * `InputSystem` : Lit le clavier -> modifie `MovableComponent`.
    * `MovementSystem` : Lit `Movable` -> modifie `Position`.
    * `RenderSystem` : Lit `Position` + `Sprite` -> Convertit en Iso -> Trie -> Dessine.

### 5.2 Collisions et Physique
* [ ] **Collision Grille :** Avant de déplacer une entité vers la tuile [X, Y], vérifier `Map[X][Y].isWalkable`.
* [ ] **Mouse Picking :** Utiliser les maths inverses de la Phase 3 pour savoir sur quelle tuile la souris clique (ex: pour casser un bloc).

---

## 🛠️ Phase 6 : Outils de Développement (ImGui)

Tu ne veux pas recompiler ton code chaque fois que tu veux tester une vitesse de déplacement ou changer une texture de sol.

* [ ] Intégrer **Dear ImGui** par-dessus ta fenêtre SFML/SDL.
* [ ] Créer une fenêtre de debug "Map Editor" :
    * Afficher les coordonnées de la souris (Grille vs Écran).
    * Sélecteur de tuile (Palette).
    * Clic sur la map = Changement de l'ID de la tuile en temps réel.
* [ ] Sauvegarde : Sérialiser le tableau de tuiles en **JSON** (avec `nlohmann/json`) pour sauver/charger tes créations.

---

## 📂 Structure de Dossiers Suggérée

Une structure propre dès le début t'évitera de tout refaire dans 2 semaines.

```text
/ProjetSandbox
|-- /bin                (Exécutables compilés et DLLs)
|-- /assets             (Images, Sons, Polices)
|   |-- /textures
|   |-- /maps           (Fichiers .json)
|
|-- /src
|   |-- main.cpp        (Point d'entrée)
|   |-- Game.cpp        (Boucle principale, Gestion du temps)
|   |
|   |-- /Core           (Moteur de base)
|   |   |-- Window.cpp
|   |   |-- InputManager.cpp
|   |   |-- ResourceManager.cpp (Gestion des textures)
|   |
|   |-- /Maths
|   |   |-- IsoHelper.cpp       (Les formules de conversion 2D <-> Iso)
|   |
|   |-- /ECS
|   |   |-- Components.hpp      (Tous les structs)
|   |   |-- Systems.cpp         (Logique de mouvement, rendu)
|   |
|   |-- /World
|   |   |-- Map.cpp             (Le tableau de tuiles)
|   |   |-- Tile.hpp            (Définition d'une tuile)
|
|-- /libs               (Dépendances : SFML, EnTT, ImGui, JSON)
|-- CMakeLists.txt      (Configuration de build)
```