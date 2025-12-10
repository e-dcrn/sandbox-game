# Plan d'Implémentation du Torque (Rotation)

Ce document détaille les étapes pour passer d'un moteur physique AABB (Axis Aligned Bounding Box) à un moteur supportant la rotation (OBB - Oriented Bounding Box) et la réponse angulaire.

## Étape 1 : Nettoyage et Structures de Données
Avant de toucher à la physique, il faut préparer les données.

### Tâches
- [ ] **Mise à jour des composants :**
    - Ajouter `angle` (en radians) dans le composant **Position** ou **Transform**.
    - Ajouter `angularVelocity` (en radians/sec) dans le composant **Velocity**.
    - Ajouter `inertia` (moment d'inertie) et `invInertia` dans **RigidBody**.
- [ ] **Calcul de l'inertie** (au moment de la création dans la Factory) :
    - Si la masse est infinie/statique, `invInertia = 0`.

### Formules (Moment d'Inertie $I$)
**Cercle** (masse $m$, rayon $r$) :
$$I = \frac{1}{2} m r^2$$

**Rectangle** (masse $m$, largeur $w$, hauteur $h$) :
$$I = \frac{1}{12} m (w^2 + h^2)$$

**Inverse Inertie** :
$$invI = \frac{1}{I}$$

---

## Étape 2 : Intégration et Affichage (Debug Visuel)
Il faut voir les objets tourner avant qu'ils ne cognent.

### Tâches
- [ ] **Mise à jour de l'Integrator** (Système de mouvement) :
    - Appliquer la vitesse angulaire à l'angle à chaque frame.
- [ ] **Mise à jour du Rendu (SDL2)** :
    - Remplacer `SDL_RenderCopy` par `SDL_RenderCopyEx` pour prendre en compte l'angle.
    - *Note : SDL attend des degrés, la physique utilise des radians.*
- [ ] **Debug Visuel (Crucial)** :
    - Dessiner une ligne du centre de l'objet vers le bord (rayon) pour visualiser la rotation des cercles.

### Formules
**Intégration d'Euler** :
$$\theta_{final} = \theta_{initial} + \omega \cdot dt$$

**Conversion** :
$$Degrés = Radians \times \frac{180}{\pi}$$

---

## Étape 3 : Détection de Collision (Le plus dur)
C'est l'étape critique. Pour appliquer du torque, il ne suffit plus de savoir si ça touche, il faut savoir **où** ça touche (Point de Contact) et la normale exacte.

### Tâches
- [ ] **Cercle vs Cercle** :
    - Inchangé (le point de contact est sur la ligne reliant les centres).
- [ ] **Rectangle vs Cercle (Rotaté)** :
    - Transformer la position du cercle dans l'espace local du rectangle pour faire les calculs comme si le rectangle était aligné (AABB).
- [ ] **Rectangle vs Rectangle (OBB)** :
    - Implémenter le **SAT** (Separating Axis Theorem).

### Objectif de cette étape
Pour chaque collision, obtenir une structure **Manifold** :
1. **Normal ($n$)** : Direction de la poussée.
2. **Penetration ($d$)** : Profondeur.
3. **ContactPoint ($p$)** : Coordonnées monde du point d'impact précis.

---

## Étape 4 : Physique de la Résolution (Impulsion avec Torque)
C'est ici qu'on modifie la réponse à la collision pour faire tourner les objets.

### Concept
Une force appliquée au centre de masse ne crée pas de rotation. Une force appliquée à un point de contact $P$ crée un couple (torque) basé sur la distance $r$ (vecteur du centre de masse vers $P$).

### Algorithme de Résolution
1. Calculer les vecteurs rayons ($r_A$ et $r_B$) depuis le centre de chaque corps vers le point de contact.
2. Calculer la vitesse relative au point de contact (incluant la rotation).
3. Calculer l'impulsion scalaire $j$ (modifiée pour inclure l'inertie).
4. Appliquer l'impulsion :
    - Vitesse linéaire : $\pm j \cdot n / m$
    - Vitesse angulaire : $\pm (r \times (j \cdot n)) / I$

### Les Formules Clés

**1. Vecteurs Rayons**
$$r_A = P_{contact} - Position_A$$
$$r_B = P_{contact} - Position_B$$

**2. Vitesse relative au point de contact**
La vitesse d'un point sur un objet qui tourne est : $V + \omega \times r$.
En 2D, le "Cross Product" d'un scalaire $\omega$ et vecteur $r$ est :
$$V_{rot}.x = -\omega \cdot r.y$$
$$V_{rot}.y = \omega \cdot r.x$$

Vitesse relative globale :
$$V_{rel} = (V_B + \omega_B \times r_B) - (V_A + \omega_A \times r_A)$$

**3. Vitesse le long de la normale**
$$VelAlongNormal = V_{rel} \cdot n$$
*(Si > 0, les objets s'éloignent déjà, on arrête).*

**4. Produit Vectoriel 2D (Cross Product)**
Pour deux vecteurs $a$ et $b$ en 2D, le résultat est un scalaire (axe Z) :
$$Cross(a, b) = a.x \cdot b.y - a.y \cdot b.x$$

**5. Calcul de l'Impulsion Scalaire ($j$) avec rotation**
Note l'ajout des termes $InvInertia$ et $Cross$. $(r \times n)^2$ est le carré du produit vectoriel scalaire.
$$j = \frac{-(1 + e) \cdot VelAlongNormal}{InvMass_A + InvMass_B + \frac{(r_A \times n)^2}{I_A} + \frac{(r_B \times n)^2}{I_B}}$$

**6. Application de l'Impulsion**
$$V_A \leftarrow V_A - (j \cdot n) \cdot InvMass_A$$
$$V_B \leftarrow V_B + (j \cdot n) \cdot InvMass_B$$
$$\omega_A \leftarrow \omega_A - \frac{Cross(r_A, j \cdot n)}{I_A}$$
$$\omega_B \leftarrow \omega_B + \frac{Cross(r_B, j \cdot n)}{I_B}$$

---

## Étape 5 : Friction (Optionnel mais recommandé) + Debug
Sans friction, une balle qui touche le sol ne "roulera" pas vraiment, elle glissera en tournant bizarrement.

### Idée rapide
1. Calculer le vecteur tangent $t$ (perpendiculaire à la normale $n$).
2. Refaire le calcul de l'impulsion $j$ mais en projetant $V_{rel}$ sur $t$ au lieu de $n$.
3. Limiter cette impulsion de friction par la loi de Coulomb :
   $$Friction \le ImpulsionNormale \times CoeffFriction$$
4. Ne pas oublier de mettre à jours l'énergie mécanique dans le mode debug

---

## Checklist Debugging
- [ ] **Debug 1 :** Créer une boite statique et une boite dynamique. La dynamique doit rebondir sans tourner (si on tape au centre).
- [ ] **Debug 2 :** Si je tape le coin de la boite, elle doit se mettre à tourner.
- [ ] **Debug 3 :** Vérifier le sens de rotation. Si je tape à droite, ça tourne dans le sens anti-horaire (ou horaire selon ton repère Y).
- [ ] **Debug 4 :** Stabilité. Si je pose une boite sur le sol, elle ne doit pas "vibrer" ou gagner de l'énergie infinie en tournant.