# TP : Implémentation du Moteur de Collisions (Cercle vs AABB)

**Objectif :** Empêcher les balles de passer à travers les murs et les faire rebondir de manière réaliste.

## 1. Architecture des Données (Components)
- [ ] Dans `src/physics/Components.hpp`, créer une `enum class ColliderType` avec les valeurs `CIRCLE` et `BOX`.
- [ ] Créer la structure `Collider`. Elle doit contenir :
    - Le `type` (l'enum créé au-dessus).
    - Les dimensions : `radius` (pour le cercle), `width` et `height` (pour la boite).
    - *Astuce : Tu peux utiliser des fonctions statiques helpers comme `createBox(w, h)` pour faciliter la construction.*

## 2. La Bibliothèque Mathématique (Collision.hpp)
- [ ] Créer `src/physics/Collision.hpp`.
- [ ] Définir une structure de retour `Manifold` (ou `CollisionResult`) contenant :
    - `bool is_colliding` : Y a-t-il contact ?
    - `float normal_x, normal_y` : Le vecteur normalisé qui indique la direction pour sortir du mur.
    - `float depth` : La profondeur de pénétration (de combien de pixels on est rentré dans le mur).
- [ ] Implémenter la fonction de détection `check_circle_aabb(...)`.
    - **Étape A :** Trouver le point du rectangle le plus proche du centre du cercle.
        - *Formule :* `closestX = clamp(circleX, boxMinX, boxMaxX)` (idem pour Y).
    - **Étape B :** Calculer le vecteur distance entre ce point proche et le centre du cercle (`distX`, `distY`).
    - **Étape C :** Comparer la longueur de ce vecteur au rayon du cercle (utiliser la distance au carré pour éviter la racine carrée si possible pour le check).
    - **Étape D :** Si collision, remplir le `Manifold`.
        - La normale est le vecteur distance normalisé.
        - Le depth est `radius - distance`.

## 3. Le Système de Résolution (PhysicsSystem)
- [ ] Dans `src/physics/PhysicsSystem.cpp`, créer une fonction `solve_collisions(registry)`.
- [ ] **La Boucle :** Itérer sur les entités dynamiques (qui ont `Velocity`, `RigidBody`, `Collider`) et les comparer aux entités statiques (qui ont `Position`, `Collider`).
- [ ] **Filtrage :** Vérifier les types. Pour l'instant : Si Dynamique == CIRCLE et Statique == BOX -> Appeler ta fonction mathématique.
- [ ] **Réponse Physique 1 (Position) :** Si collision détectée (`manifold.is_colliding` est vrai) :
    - Repousser l'entité dynamique hors du mur.
    - *Formule :* `pos += normal * depth`.
- [ ] **Réponse Physique 2 (Vitesse / Rebond) :**
    - Calculer la vitesse relative par rapport à la normale (Produit Scalaire : `dot(velocity, normal)`).
    - Si le résultat est > 0, l'objet s'éloigne déjà du mur, ne rien faire (évite le collage).
    - Sinon, calculer le vecteur de réflexion.
    - *Formule de réflexion :* `v_new = v_old - (1 + restitution) * (v_old · normal) * normal`.
    - *Note :* `restitution` vient du composant `RigidBody` (0 = mou, 1 = rebond parfait).

## 4. Intégration
- [ ] Appeler `solve_collisions` dans `PhysicsSystem::update` **après** l'application des forces et l'intégration du mouvement.
- [ ] Mettre à jour `EntityFactory.hpp` pour ajouter des composants `Collider` aux balles (Circle) et aux murs (Box).

## Vérification
- [ ] Les balles ne traversent plus le mur.
- [ ] Les balles rebondissent correctement (angle d'entrée = angle de sortie).
- [ ] Les balles ne restent pas collées au mur si elles le touchent doucement.