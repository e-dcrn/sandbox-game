# TODO

## Current :
1. Factoriser la physique
- [x] Séparer les fonction dans PhysicsSystem,PhysicsMath et Collision
- [x] Utiliser les namespace
- [x] Renomer les fonctions et structures si nécéssaire
- [x] Créer des nouvelle fonction mathématique pour simplifier PhysicsSystem ou Collision
2. [x] Implémenter AABB pour le partitionnement spacial
3. [ ] Checker les points de contact qui peuvent encore poser problème (test)
4. [ ] Ne pas oublier de mettre à jours l'énergie mécanique dans le mode debug
5. [ ] Réparer le RayCast

## Frotement solide :
### Idée rapide
1. Calculer le vecteur tangent $t$ (perpendiculaire à la normale $n$).
2. Refaire le calcul de l'impulsion $j$ mais en projetant $V_{rel}$ sur $t$ au lieu de $n$.
3. Limiter cette impulsion de friction par la loi de Coulomb :
   $$Friction \le ImpulsionNormale \times CoeffFriction$$

---

## Bonus :
- [ ] Implémenter les polygones (attention au point de contact qui est technique)

---

## Checklist Debugging
- [ ] **Debug 1 :** Créer une boite statique et une boite dynamique. La dynamique doit rebondir sans tourner (si on tape au centre).
- [ ] **Debug 2 :** Si je tape le coin de la boite, elle doit se mettre à tourner.
- [ ] **Debug 3 :** Vérifier le sens de rotation. Si je tape à droite, ça tourne dans le sens anti-horaire (ou horaire selon ton repère Y).
- [ ] **Debug 4 :** Stabilité. Si je pose une boite sur le sol, elle ne doit pas "vibrer" ou gagner de l'énergie infinie en tournant.