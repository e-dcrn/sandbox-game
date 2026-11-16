#include <iostream>
#include <entt/entt.hpp>
#include <SDL.h>

#include "TestUtils.hpp"
#include "../physics/PhysicsSystem.hpp" 
#include "../core/EntityBuilder.hpp" 

// --- HELPERS ---
float VecLen(Vect2 v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// ================================================================================================
// TEST 1 : Maths de Collision (Tests unitaires des fonctions statiques)
// ================================================================================================
void Test_Maths_Circle_Circle() {
    std::cout << "  > Testing Math: Circle vs Circle (Extended)..." << std::endl;
    
    // -------------------------------------------------------------------------
    // 1. Cas Standard (Chevauchement Axe X)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {0, 0};
        Vect2 c2 = {15, 0}; 
        // Rayons 10. Distance 15. Overlap = 20 - 15 = 5.
        Collision::Manifold res = Collision::test_circles(c1, 10.0f, c2, 10.0f);
        
        ExpectTrue(res.is_colliding, "[Math] Standard Overlap");
        ExpectTrue(IsClose(res.depth, 5.0f), "[Math] Depth Correct");
        // Normale doit repousser c2 vers la droite (1, 0) par rapport à c1
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[Math] Normal X");
    }

    // -------------------------------------------------------------------------
    // 2. Juste au bord (Touching / Tangent)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {0, 0};
        Vect2 c2 = {20, 0}; 
        // Rayons 10. Distance 20. Overlap = 0.
        // Check for overlap behavior
        Collision::Manifold res = Collision::test_circles(c1, 10.0f, c2, 10.0f);
        
        ExpectTrue(!res.is_colliding, "[Math] Touching (No Penetration)");
    }

    // -------------------------------------------------------------------------
    // 3. Pas de Collision (Loin)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {0, 0};
        Vect2 c2 = {30, 0}; 
        Collision::Manifold res = Collision::test_circles(c1, 10.0f, c2, 10.0f);
        ExpectTrue(!res.is_colliding, "[Math] Far Away");
    }

    // -------------------------------------------------------------------------
    // 4. Chevauchement Vertical (Axe Y)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {0, 0};
        Vect2 c2 = {0, 15}; // Au-dessus
        Collision::Manifold res = Collision::test_circles(c1, 10.0f, c2, 10.0f);
        
        ExpectTrue(res.is_colliding, "[Math] Vertical Overlap");
        ExpectTrue(IsClose(res.depth, 5.0f), "[Math] Vertical Depth");
        // La normale doit être (0, 1)
        ExpectTrue(IsVecClose(res.normal_vector, {0.0f, 1.0f}), "[Math] Normal Y");
    }

    // -------------------------------------------------------------------------
    // 5. Inclusion Totale (Un petit dans un grand)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {0, 0}; // Grand (R=20)
        Vect2 c2 = {5, 0}; // Petit (R=5)
        // Le petit est à x=5. Son bord gauche est à 0, bord droit à 10.
        // Il est totalement DANS le grand.
        // Distance centres = 5. Rayons cumulés = 25.
        // Profondeur = 25 - 5 = 20.
        Collision::Manifold res = Collision::test_circles(c1, 20.0f, c2, 5.0f);
        
        ExpectTrue(res.is_colliding, "[Math] Full Containment");
        ExpectTrue(IsClose(res.depth, 20.0f), "[Math] Containment Depth");
        // La normale doit toujours pousser le petit vers l'extérieur le plus proche
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[Math] Containment Normal");
    }

    // -------------------------------------------------------------------------
    // 6. Même Position Exacte (Singularité / NaN Check)
    // -------------------------------------------------------------------------
    {
        Vect2 c1 = {10, 10};
        Vect2 c2 = {10, 10};
        // Distance = 0. Division par zéro potentielle pour la normale !
        Collision::Manifold res = Collision::test_circles(c1, 10.0f, c2, 10.0f);
        
        ExpectTrue(res.is_colliding, "[Math] Same Position");
        ExpectTrue(IsClose(res.depth, 20.0f), "[Math] Same Position Depth"); // Rayon A + Rayon B
        
        // Vérifie que la normale n'est pas NaN
        bool normalIsValid = !std::isnan(res.normal_vector.x) && !std::isnan(res.normal_vector.y);
        ExpectTrue(normalIsValid, "[Math] Same Position Normal is Valid (No NaN)");
        
        // Vérifie si la normale a une longueur de 1 (ou 0 si géré ainsi, mais 1 est préférable)
        float len = std::sqrt(res.normal_vector.x*res.normal_vector.x + res.normal_vector.y*res.normal_vector.y);
        ExpectTrue(IsClose(len, 1.0f) || IsClose(len, 0.0f), "[Math] Same Position Normal Normalized");
    }
}

void Test_Maths_Box_Box() {
    std::cout << "  > Testing Math: Box vs Box (Exhaustive)..." << std::endl;

    // -------------------------------------------------------------------------
    // 1. Collision Frontale (Axe X)
    // -------------------------------------------------------------------------
    {
        Vect2 b1_pos = {0, 0};   Vect2 b1_size = {10, 10};
        Vect2 b2_pos = {8, 0};   Vect2 b2_size = {10, 10}; // Dist 8. Overlap = 10-8=2.
        
        Collision::Manifold res = Collision::test_boxes(b1_pos, b1_size, b2_pos, b2_size);
        ExpectTrue(res.is_colliding, "[Math] X Overlap");
        ExpectTrue(IsClose(res.depth, 2.0f), "[Math] X Depth");
        ExpectTrue(std::abs(res.normal_vector.x) > 0.9f, "[Math] Normal X");
    }

    // -------------------------------------------------------------------------
    // 2. Collision Verticale (Axe Y)
    // -------------------------------------------------------------------------
    {
        Vect2 b1_pos = {0, 0};   Vect2 b1_size = {10, 10};
        Vect2 b2_pos = {0, 9};   Vect2 b2_size = {10, 10}; // Overlap = 1.
        
        Collision::Manifold res = Collision::test_boxes(b1_pos, b1_size, b2_pos, b2_size);
        ExpectTrue(res.is_colliding, "[Math] Y Overlap");
        ExpectTrue(IsClose(res.depth, 1.0f), "[Math] Y Depth");
        ExpectTrue(std::abs(res.normal_vector.y) > 0.9f, "[Math] Normal Y");
    }

    // -------------------------------------------------------------------------
    // 3. SAT Logic (Moindre Pénétration)
    // Overlap X = 1.0 (Petit) vs Overlap Y = 8.0 (Grand)
    // -> Doit choisir X.
    // -------------------------------------------------------------------------
    {
        Vect2 b1_pos = {0, 0};   Vect2 b1_size = {10, 10};
        Vect2 b2_pos = {9, 2};   Vect2 b2_size = {10, 10}; 

        Collision::Manifold res = Collision::test_boxes(b1_pos, b1_size, b2_pos, b2_size);
        
        ExpectTrue(res.is_colliding, "[Math] SAT Mixed");
        ExpectTrue(IsClose(res.depth, 1.0f), "[Math] SAT picked smallest depth (X)");
        ExpectTrue(std::abs(res.normal_vector.x) > 0.9f, "[Math] SAT Normal is X");
    }

    // -------------------------------------------------------------------------
    // 4. Inclusion Totale (Petite boîte DANS Grande boîte)
    // Grande : 20x20. Petite : 2x2.
    // Position Petite : (5, 0).
    // Elle est totalement dedans.
    // Overlap X : (10 + 1) - 5 = 6. (Distance au bord droit)
    // Overlap Y : (10 + 1) - 0 = 11. (Distance au bord haut/bas)
    // -> Le chemin le plus court pour sortir est X (6 < 11).
    // -------------------------------------------------------------------------
    {
        Vect2 big_pos = {0, 0};   Vect2 big_size = {20, 20};
        Vect2 small_pos = {5, 0}; Vect2 small_size = {2, 2};

        Collision::Manifold res = Collision::test_boxes(big_pos, big_size, small_pos, small_size);

        ExpectTrue(res.is_colliding, "[Math] Full Inclusion");
        // Profondeur attendue : Overlap X = (10+1) - 5 = 6.
        ExpectTrue(IsClose(res.depth, 6.0f), "[Math] Inclusion Depth (shortest path)");
        // Normale : Doit pousser vers le bord le plus proche (Droite, donc X+)
        ExpectTrue(res.normal_vector.x > 0.9f, "[Math] Inclusion Normal pushes out");
    }

    // -------------------------------------------------------------------------
    // 5. Même Position Exacte (Singularité)
    // Deux boîtes identiques au même endroit.
    // Overlap X = 10. Overlap Y = 10.
    // Le code doit faire un choix arbitraire mais valide (souvent Y dans l'implémentation standard).
    // -------------------------------------------------------------------------
    {
        Vect2 b1_pos = {10, 10}; Vect2 b1_size = {10, 10};
        Vect2 b2_pos = {10, 10}; Vect2 b2_size = {10, 10};
        
        Collision::Manifold res = Collision::test_boxes(b1_pos, b1_size, b2_pos, b2_size);
        
        ExpectTrue(res.is_colliding, "[Math] Same Position");
        ExpectTrue(IsClose(res.depth, 10.0f), "[Math] Same Position Depth (Half Width/Height)");
        
        // Vérifie juste que la normale est valide (longueur 1)
        float len = std::sqrt(res.normal_vector.x*res.normal_vector.x + res.normal_vector.y*res.normal_vector.y);
        ExpectTrue(IsClose(len, 1.0f), "[Math] Same Position Normal Normalized");
    }

    // -------------------------------------------------------------------------
    // 6. Juste au bord (Touching)
    // -------------------------------------------------------------------------
    {
        Vect2 b1_pos = {0, 0};   Vect2 b1_size = {10, 10};
        Vect2 b2_pos = {0, 10};  Vect2 b2_size = {10, 10}; // Overlap = 0
        
        Collision::Manifold res = Collision::test_boxes(b1_pos, b1_size, b2_pos, b2_size);
        ExpectTrue(!res.is_colliding, "[Math] Touching (No Penetration)");
    }
}

void Test_Maths_Circle_Box() {
    std::cout << "  > Testing Math: Circle vs Box (Exhaustive)..." << std::endl;

    Vect2 box_pos = {0, 0}; 
    Vect2 box_size = {20, 20}; // Extents +/- 10
    float radius = 5.0f;

    // -------------------------------------------------------------------------
    // 1. Collision Frontale (Face X)
    // -------------------------------------------------------------------------
    {
        // Cercle à droite (x=14). Bord Box à x=10.
        // Distance surface = 4. Rayon = 5. Overlap = 1.
        Vect2 circ_pos = {14, 0}; 
        
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);
        ExpectTrue(res.is_colliding, "[Math] Face X Overlap");
        ExpectTrue(IsClose(res.depth, 1.0f), "[Math] Face X Depth");
        // Normale : Doit repousser vers la droite (1, 0)
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[Math] Face X Normal");
    }

    // -------------------------------------------------------------------------
    // 2. Collision Frontale (Face Y)
    // -------------------------------------------------------------------------
    {
        // Cercle au-dessus (y=13). Bord Box à y=10.
        // Distance surface = 3. Rayon = 5. Overlap = 2.
        Vect2 circ_pos = {0, 13}; 
        
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);
        ExpectTrue(res.is_colliding, "[Math] Face Y Overlap");
        ExpectTrue(IsClose(res.depth, 2.0f), "[Math] Face Y Depth");
        // Normale : Doit repousser vers le haut (0, 1)
        ExpectTrue(IsVecClose(res.normal_vector, {0.0f, 1.0f}), "[Math] Face Y Normal");

        std::cout << "    -> Corner Vel Result: " << res.normal_vector.x << ", " << res.normal_vector.y << std::endl;

    }

    // -------------------------------------------------------------------------
    // 3. Collision Coin (Corner) - CRUCIAL
    // Le cercle tape le coin (10, 10) en diagonale.
    // Position du cercle : (13, 14).
    // Point le plus proche sur la box : (10, 10).
    // Vecteur diff : (3, 4). Distance : sqrt(9+16) = 5.
    // Rayon = 5.
    // C'est un cas limite (Tangent). On le rapproche un peu pour avoir du depth.
    // Pos : (12.5, 13). Closest (10, 10). Diff (2.5, 3).
    // DistSq = 6.25 + 9 = 15.25. Dist = 3.905.
    // Depth = 5 - 3.905 = 1.095.
    // -------------------------------------------------------------------------
    {
        Vect2 circ_pos = {12.5f, 13.0f};
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);

        ExpectTrue(res.is_colliding, "[Math] Corner Overlap");
        
        // Vérifie que la profondeur est positive (~1.1)
        ExpectTrue(res.depth > 0.0f, "[Math] Corner Depth > 0");

        // Vérifie que la normale est diagonale (ni purement X, ni purement Y)
        bool isDiagonal = (res.normal_vector.x > 0.1f) && (res.normal_vector.y > 0.1f);
        ExpectTrue(isDiagonal, "[Math] Corner Normal is Diagonal");

        std::cout << "    -> Corner Vel Result: " << res.normal_vector.x << ", " << res.normal_vector.y << std::endl;

    }

    // -------------------------------------------------------------------------
    // 4. À l'intérieur (Inside)
    // Le centre du cercle est DANS la boîte.
    // Cercle à (8, 0). Rayon 5.
    // Dist bord Droit (x=10) = 2. Dist bord Gauche (x=-10) = 18.
    // -> Doit sortir par la Droite.
    // Profondeur = DistToEdge + Radius = 2 + 5 = 7.
    // (Car il faut parcourir 2 pour sortir le centre, + 5 pour sortir le bord du cercle).
    // -------------------------------------------------------------------------
    {
        Vect2 circ_pos = {8, 0};
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);

        ExpectTrue(res.is_colliding, "[Math] Inside Box");
        ExpectTrue(IsClose(res.depth, 7.0f), "[Math] Inside Depth (DistToEdge + Radius)");
        // Normale : (1, 0)
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[Math] Inside Normal");
    }

    // -------------------------------------------------------------------------
    // 5. Tangent (Touching)
    // -------------------------------------------------------------------------
    {
        Vect2 circ_pos = {15, 0};
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);
        
        ExpectTrue(res.is_colliding, "[Math] Touching Edge");
        ExpectTrue(IsClose(res.depth, 0.0f), "[Math] Touching Depth is 0");
    }

    // -------------------------------------------------------------------------
    // 6. Pas de Collision
    // -------------------------------------------------------------------------
    {
        Vect2 circ_pos = {20, 20}; 
        Collision::Manifold res = Collision::test_circle_box(circ_pos, radius, box_pos, box_size);
        ExpectTrue(!res.is_colliding, "[Math] No Overlap");
    }
}

void Test_Maths_Circle_RotatedBox() {
    std::cout << "  > Testing Math: Circle vs Rotated Box (8 Cases)..." << std::endl;
    float pi = 3.1415926535f;

    // --- 1. Rotation 0° (Référence AABB) ---
    {
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {20, 10}; float angle = 0.0f;
        Vect2 circlePos = {12, 0}; float radius = 5.0f;
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, radius, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[1] 0 deg - Standard");
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[1] Normal X");
    }

    // --- 2. Rotation 90° (La largeur devient hauteur) ---
    {
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {20, 10}; float angle = pi / 2.0f; 
        Vect2 circlePos = {0, 12}; float radius = 5.0f; // Tape le "haut" (anciennement coté droit)
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, radius, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[2] 90 deg - Top Hit");
        ExpectTrue(IsVecClose(res.normal_vector, {0.0f, 1.0f}), "[2] Normal Y");
    }

    // --- 3. Rotation 45° (Face Diagonale) ---
    {
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {20, 20}; float angle = pi / 4.0f;
        float dist = 15.0f; 
        Vect2 circlePos = { dist * 0.7071f, dist * 0.7071f }; 
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, 10.0f, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[3] 45 deg - Face");
        // La normale doit être diagonale (0.7, 0.7)
        ExpectTrue(res.normal_vector.x > 0.5f && res.normal_vector.y > 0.5f, "[3] Normal Diag");
    }

    // --- 4. Coin (Corner Hit) avec Rotation ---
    {
        Vect2 boxPos = {100, 100}; Vect2 boxDim = {20, 20}; float angle = pi / 4.0f;
        // Le coin local (10,10) tourné de 45° est à (0, 14.14) par rapport au centre.
        Vect2 circlePos = {100.0f, 114.14f + 2.0f}; // Juste au dessus du coin
        float radius = 5.0f;
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, radius, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[4] Corner Hit");
        // La normale du coin (1,1) local devient (0,1) monde après rotation 45°
        ExpectTrue(IsVecClose(res.normal_vector, {0.0f, 1.0f}), "[4] Corner Normal Transformed");
    }

    // --- 5. Angle Négatif (-45°) ---
    {
        // Boite tournée vers la droite-bas
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {20, 20}; float angle = -pi / 4.0f;
        Vect2 circlePos = {10, -10}; // Dans le quadrant bas-droit
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, 5.0f, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[5] Negative Angle");
    }

    // --- 6. Inclusion Totale (Cercle DANS la boite) ---
    {
        // Centre sur centre
        Vect2 boxPos = {10, 10}; Vect2 boxDim = {20, 10}; float angle = 1.0f; // Angle arbitraire
        Vect2 circlePos = {10, 10}; float radius = 2.0f;
        
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, radius, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[6] Full Inclusion");
        // Profondeur : min(halfW, halfH) + radius = 5 + 2 = 7.
        ExpectTrue(IsClose(res.depth, 7.0f), "[6] Inclusion Depth");
    }

    // --- 7. Boîte très fine (Mur) ---
    {
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {100, 1}; float angle = pi / 2.0f; // Mur vertical fin
        Vect2 circlePos = {2.0f, 0.0f}; float radius = 2.0f; // Touche le mur
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, radius, boxPos, boxDim, angle);
        ExpectTrue(res.is_colliding, "[7] Thin Wall");
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[7] Thin Wall Normal");
    }

    // --- 8. Loin (Pas de collision) ---
    {
        Vect2 boxPos = {0, 0}; Vect2 boxDim = {10, 10}; float angle = pi / 3.0f;
        Vect2 circlePos = {50, 50}; 
        Collision::Manifold res = Collision::test_circle_box_rotated(circlePos, 5.0f, boxPos, boxDim, angle);
        ExpectTrue(!res.is_colliding, "[8] Far Away");
    }
}

void Test_Maths_RotatedBox_Box() {
    std::cout << "  > Testing Math: Rotated Box vs Rotated Box (8 Cases)..." << std::endl;
    float pi = 3.1415926535f;

    // --- 1. Aligné (Référence) ---
    {
        Collision::Manifold res = Collision::test_boxes_rotated({0,0}, {20,20}, 0, {15,0}, {20,20}, 0);
        ExpectTrue(res.is_colliding, "[1] Aligned");
        ExpectTrue(IsVecClose(res.normal_vector, {1.0f, 0.0f}), "[1] Normal");
    }

    // --- 2. La Croix (The Cross) ---
    // Cas classique qui piège les algos simplistes.
    // A: 20x2 (Horizontale). B: 2x20 (Verticale). Centre commun.
    // Overlap X: petit. Overlap Y: petit.
    {
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {20, 2}, 0.0f,         // Barre -
            {0,0}, {2, 20}, pi / 2.0f     // Barre | (tournée 90)
        );
        ExpectTrue(res.is_colliding, "[2] The Cross (+)");
        // L'overlap est le même sur les 4 axes (c'est un carré 2x2 au milieu).
        // La profondeur doit être 1.0 (half width du petit coté) + 1.0 = 2.0 ? 
        // Non, SAT cherche à sortir le plus vite. Sortir par X (dist 1) ou Y (dist 1).
        ExpectTrue(res.depth > 0.0f, "[2] Cross Depth found");
    }

    // --- 3. Diamant sur le Sol (45°) ---
    {
        // Sol (40x10) vs Cube tourné (20x20)
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {40,10}, 0.0f,
            {0,10}, {20,20}, pi/4.0f 
        );
        ExpectTrue(res.is_colliding, "[3] Diamond/Floor");
        // La normale doit repousser vers le HAUT (Axe Y du sol) car c'est le séparateur le plus fort
        ExpectTrue(IsClose(std::abs(res.normal_vector.y), 1.0f), "[3] Floor Normal dominates");
    }

    // --- 4. Presque Parallèles (1°) - Test de précision ---
    // Souvent les produits scalaires foirent quand les axes sont presque colinéaires
    {
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {20,20}, 0.0f,
            {15,0.5f}, {20,20}, 0.017f // ~1 degré
        );
        ExpectTrue(res.is_colliding, "[4] Almost Parallel");
        ExpectTrue(IsClose(std::abs(res.normal_vector.x), 1.0f), "[4] Normal X dominates");
    }

    // --- 5. Inclusion Totale Rotative ---
    // Petite boite tournée DANS une grande boite droite
    {
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {100,100}, 0.0f,
            {0,0}, {10,10}, pi/3.0f
        );
        ExpectTrue(res.is_colliding, "[5] Full Inclusion");
        // La normale doit pousser vers le bord le plus proche (disons X ou Y, peu importe, mais valide)
        ExpectTrue(res.depth > 40.0f, "[5] Deep Depth"); // Doit sortir de ~50 unités
    }

    // --- 6. Coins qui se touchent (Kissing Corners) ---
    // Très dur à détecter. On fait un léger overlap pour être sûr.
    {
        // A: 0,0 (20x20). Coin droit-haut: (10,10).
        // B: 20,20 (20x20). Coin gauche-bas: (10,10).
        // On rapproche B un tout petit peu: (19.8, 19.8)
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {20,20}, 0.0f,
            {19.5f, 19.5f}, {20,20}, 0.0f
        );
        ExpectTrue(res.is_colliding, "[6] Corner Kiss");
        // La normale devrait être diagonale ou un des axes principaux selon l'implémentation
        // SAT favorise souvent un axe principal s'il y a égalité, mais ici on attend juste true.
    }

    // --- 7. Séparation par Rotation (Le ventilateur) ---
    // Si B ne tournait pas, il toucherait. Mais il tourne et l'espace se libère.
    {
        // A: (0,0) 20x2. B: (12, 0) 20x2.
        // Si B à 0°, ça touche (overlap X).
        // Si B à 90°, c'est une barre verticale à x=12. 
        // Largeur B devient 2. Étendue X: 11 à 13.
        // Bord droit A: 10. -> Pas de collision (11 > 10).
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {20,2}, 0.0f,
            {12,0}, {20,2}, pi/2.0f
        );
        ExpectTrue(!res.is_colliding, "[7] Separated by 90deg rotation");
    }

    // --- 8. Gigantisme (Échelle) ---
    // Une boite géante vs une minuscule. Test de précision float.
    {
        Collision::Manifold res = Collision::test_boxes_rotated(
            {0,0}, {10000, 10000}, 0.1f,
            {5000, 0}, {1, 1}, 0.0f
        );
        // Le bord est vers 5000. La petite boite est à 5000. Ça doit toucher.
        ExpectTrue(res.is_colliding, "[8] Huge vs Tiny");
    }
}

void Test_Collision_Maths() {
    std::cout << "--- Testing Collision Maths (All Types) ---" << std::endl;
    Test_Maths_Circle_Circle();
    Test_Maths_Box_Box();
    Test_Maths_Circle_Box();
    Test_Maths_Circle_RotatedBox();
    Test_Maths_RotatedBox_Box();
}

// ================================================================================================
// TEST 2 : Système Circle vs Circle 
// ================================================================================================
void Test_System_Circle_vs_Circle() {
    std::cout << "\n--- Testing System: Circle vs Circle (Detailed) ---" << std::endl;

    // ==============================================================================
    // SCÉNARIO 1 : Le Billard (Choc Elastique Parfait, Masses Égales)
    // Balle A (V=10) tape Balle B (V=0).
    // Attendu : A s'arrête (presque), B part avec la vitesse de A.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        auto e1 = builder.asCircle(10).at(-10, 0)
            .withVelocity(50, 0).withMass(1.0f).withPhysicsMaterial(1.0f) // Restitution 1.0
            .withCollider().build();

        auto e2 = builder.asCircle(10).at(10, 0)
            .withVelocity(0, 0).withMass(1.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // On FORCE le chevauchement (Overlap de 2 pixels)
        registry.replace<Position>(e1, Position{-9.0f, 0.0f,0.0f});
        registry.replace<Position>(e2, Position{ 9.0f, 0.0f,0.0f});

        physics.update(registry, 0.016f);

        auto v1 = registry.get<Velocity>(e1);
        auto v2 = registry.get<Velocity>(e2);

        // A a transféré son énergie, elle doit être proche de 0 ou reculer légèrement
        ExpectTrue(v1.x <= 2.0f, "[1] Billiard: Ball A transferred energy");
        // B doit avoir gagné beaucoup de vitesse vers la droite
        ExpectTrue(v2.x > 8.0f,  "[1] Billiard: Ball B gained energy");
        
        std::cout << "    -> Result: VelA=" << v1.x << " VelB=" << v2.x << std::endl;
    }

    // ==============================================================================
    // SCÉNARIO 2 : Collision Verticale (Axe Y)
    // Vérifie que tes maths (dot product, normales) marchent aussi en vertical.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // Balle HAUT (descend)
        auto top = builder.asCircle(10).at(0, -11)
            .withVelocity(0, 10).withMass(1.0f).withCollider().build();

        // Balle BAS (monte)
        auto bot = builder.asCircle(10).at(0, 11)
            .withVelocity(0, -10).withMass(1.0f).withCollider().build();

        // Force Overlap Y
        registry.replace<Position>(top, Position{0.0f, -9.0f, 0.0f});
        registry.replace<Position>(bot, Position{0.0f,  9.0f, 0.0f});

        physics.update(registry, 0.016f);

        auto vTop = registry.get<Velocity>(top);
        auto vBot = registry.get<Velocity>(bot);

        ExpectTrue(vTop.y < 0, "[2] Vertical: Top ball bounced up");
        ExpectTrue(vBot.y > 0, "[2] Vertical: Bot ball bounced down");
    }

    // ==============================================================================
    // SCÉNARIO 3 : David vs Goliath (Différence de Masse)
    // Une balle LOURDE (10kg) tape une balle LÉGÈRE (1kg) à l'arrêt.
    // Attendu : La lourde continue d'avancer (ne rebondit pas en arrière !).
    //           La légère part très vite.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // GOLIATH (10kg, V=10)
        auto heavy = builder.asCircle(10).at(-10, 0)
            .withVelocity(50, 0).withMass(10.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // DAVID (1kg, V=0)
        auto light = builder.asCircle(10).at(10, 0)
            .withVelocity(0, 0).withMass(1.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // Force Overlap
        registry.replace<Position>(heavy, Position{-9.0f, 0.0f, 0.0f});
        registry.replace<Position>(light, Position{ 9.0f, 0.0f, 0.0f});

        physics.update(registry, 0.016f);

        auto vHeavy = registry.get<Velocity>(heavy);
        auto vLight = registry.get<Velocity>(light);

        // La lourde doit continuer d'avancer (V > 0) car elle a plus d'inertie
        ExpectTrue(vHeavy.x > 0, "[3] Mass: Heavy object kept going forward");
        // La légère doit partir plus vite que la lourde (Conservation momentum)
        ExpectTrue(vLight.x > vHeavy.x, "[3] Mass: Light object shot forward");

        std::cout << "    -> Result: HeavyVel=" << vHeavy.x << " LightVel=" << vLight.x << std::endl;
    }
}

// ================================================================================================
// TEST 3 : Système Box vs Box
// ================================================================================================
void Test_System_Box_vs_Box() {
    std::cout << "\n--- Testing System: Box vs Box (Detailed) ---" << std::endl;

    // ==============================================================================
    // SCÉNARIO 1 : Choc Frontal X (Le Classique)
    // Vérifie le rebond simple gauche/droite.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        auto e1 = builder.asBox(20, 20).at(-11, 0)
            .withVelocity(10, 0).withMass(1.0f).withPhysicsMaterial(1.0f).withCollider().build();

        auto e2 = builder.asBox(20, 20).at(11, 0)
            .withVelocity(-10, 0).withMass(1.0f).withPhysicsMaterial(1.0f).withCollider().build();

        // Force Overlap X (2 pixels)
        registry.replace<Position>(e1, Position{-9.0f, 0.0f, 0.0f});
        registry.replace<Position>(e2, Position{ 9.0f, 0.0f, 0.0f});

        physics.update(registry, 0.016f);

        auto v1 = registry.get<Velocity>(e1);
        ExpectTrue(v1.x < 0, "[1] Frontal X: Box A bounced left");
    }

    // ==============================================================================
    // SCÉNARIO 2 : Choc Vertical Y (Le "Sol")
    // Une boîte tombe sur une autre qui monte. Vérifie l'axe Y.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // Boîte du HAUT (Tombe)
        auto top = builder.asBox(20, 20).at(0, -11)
            .withVelocity(0, 10).withMass(1.0f).withPhysicsMaterial(1.0f).withCollider().build();

        // Boîte du BAS (Monte)
        auto bot = builder.asBox(20, 20).at(0, 11)
            .withVelocity(0, -10).withMass(1.0f).withPhysicsMaterial(1.0f).withCollider().build();

        // Force Overlap Y (2 pixels)
        registry.replace<Position>(top, Position{0.0f, -9.0f, 0.0f});
        registry.replace<Position>(bot, Position{0.0f,  9.0f, 0.0f});

        physics.update(registry, 0.016f);

        auto vTop = registry.get<Velocity>(top);
        
        // La boîte du haut doit repartir vers le haut (Vitesse Y négative ou nulle)
        ExpectTrue(vTop.y < 0, "[2] Vertical Y: Top Box bounced up");
    }

    // ==============================================================================
    // SCÉNARIO 3 : Mur Statique (Mass 0)
    // Une boîte tape un mur immobile. Le mur ne doit PAS bouger.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // LE MUR (Statique)
        auto wall = builder.asBox(20, 100).at(20, 0)
            .withMass(0.0f) // Masse Infinie
            .withPhysicsMaterial(1.0f)
            .withCollider().build();

        // LA BOITE (Dynamique)
        auto box = builder.asBox(20, 20).at(0, 0)
            .withVelocity(50, 0).withMass(1.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // Force Overlap (La boite rentre dans le mur)
        // Mur à x=20 (bord gauche à 10). Boite largeur 20 (bord droit à x+10).
        // Si boite à x=2, bord droit à 12. Overlap de 2 pixels.
        registry.replace<Position>(box, Position{2.0f, 0.0f, 0.0f});

        physics.update(registry, 0.016f);

        auto vBox = registry.get<Velocity>(box);
        auto vWall = registry.get<Velocity>(wall); // Devrait rester (0,0) si tu as un composant Velocity

        ExpectTrue(vBox.x < 0, "[3] Static Wall: Box bounced back");
        // Si le mur a une vélocité (même ignorée par la physique), elle doit rester 0
        if(registry.all_of<Velocity>(wall)) {
             ExpectTrue(vWall.x == 0.0f, "[3] Static Wall: Wall didn't move");
        }
    }

    // ==============================================================================
    // SCÉNARIO 4 : Le "Glancing Blow" (Test du SAT / Moindre Pénétration)
    // C'est le test le plus important pour les AABB.
    // Deux boîtes se rentrent dedans par le coin.
    // - Pénétration X = 1 pixel (Touche à peine sur le côté)
    // - Pénétration Y = 10 pixels (Gros chevauchement vertical)
    // -> La physique DOIT résoudre sur X (le plus petit chemin de sortie).
    // Si elle résout sur Y, la boîte va sauter au plafond magiquement (Bug fréquent).
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // Boite A (0,0) 20x20
        auto e1 = builder.asBox(20, 20).at(0, 0)
            .withVelocity(10, 0).withMass(1.0f).withCollider().build();

        // Boite B (19, 5) 20x20. 
        // Centre X dist = 19. Largeurs cumulées = 20. Overlap X = 1.
        // Centre Y dist = 5.  Hauteurs cumulées = 20. Overlap Y = 15.
        // VERDICT : Overlap X (1) < Overlap Y (15). La normale DOIT être horizontale (X).
        builder.asBox(20, 20).at(19, 5) 
            .withVelocity(-10, 0).withMass(1.0f).withCollider().build();

        // Pas besoin de force replace ici, les positions initiales créent déjà l'overlap voulu
        physics.update(registry, 0.016f);

        auto v1 = registry.get<Velocity>(e1);
        
        // Si la résolution est correcte (X), e1 rebondit vers la gauche.
        // Si la résolution est fausse (Y), e1 garderait sa vitesse X et partirait vers le haut/bas.
        ExpectTrue(v1.x < 0, "[4] SAT Logic: Resolved on X axis (smallest overlap)");
        
        // Vérifions qu'on n'a pas créé de vitesse Y magique (ou très peu due à la friction)
        ExpectTrue(std::abs(v1.y) < 1.0f, "[4] SAT Logic: No magical Y bounce");
    }
}

// ================================================================================================
// TEST 4 : Système Circle vs Box (Statique)
// ================================================================================================
void Test_System_Circle_vs_Box() {
    std::cout << "\n--- Testing System: Circle vs Box (Detailed) ---" << std::endl;

    // ==============================================================================
    // SCÉNARIO 1 : Le Mur (Face Collision - Axe X)
    // Cas classique : Rebond sur une surface plane verticale.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // MUR (Statique) : x=10, Largeur 10. Bord Gauche à x=5.
        auto wall = builder.asBox(10.0f, 100.0f).at(10.0f, 0.0f)
            .withMass(0.0f).withPhysicsMaterial(1.0f).withCollider().build();

        // BALLE (Dynamique) : Rayon 2.
        auto ball = builder.asCircle(2.0f).at(0.0f, 0.0f)
            .withVelocity(100.0f, 0.0f).withMass(1.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // Force Overlap : Balle à x=4. Bord mur à x=5. Rayon 2.
        // La balle pénètre de 1 pixel.
        registry.replace<Position>(ball, Position{4.0f, 0.0f, 0.0f});

        physics.update(registry, 0.016f);

        const auto& vBall = registry.get<Velocity>(ball);
        
        // Rebond vers la gauche attendu
        ExpectTrue(vBall.x < 0, "[1] Face X: Ball bounced off wall");
        // Pas de mouvement vertical parasite
        ExpectTrue(IsClose(vBall.y, 0.0f), "[1] Face X: No vertical deviation");

        if(registry.all_of<Velocity>(wall)) {
             const auto& vWall = registry.get<Velocity>(wall);
             ExpectTrue(vWall.x == 0 && vWall.y == 0, "[1] Wall stayed static");
        }
    }

    // ==============================================================================
    // SCÉNARIO 2 : Le Coin (Corner Collision)
    // Le test le plus dur. Le cercle tape le coin exact de la boîte.
    // La normale ne doit être ni (1,0) ni (0,1), mais diagonale (0.7, 0.7).
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // BOITE (Statique) : 20x20 centrée en (0,0).
        // Coin Haut-Droit est à (10, 10).
        auto box = builder.asBox(20.0f, 20.0f).at(0.0f, 0.0f)
            .withMass(0.0f).withPhysicsMaterial(1.0f).withCollider().build();

        // BALLE : Rayon 5.
        // On la place en diagonale du coin (10, 10).
        // Position cible : (13, 14).
        // Distance au coin : sqrt(3^2 + 4^2) = 5.
        // Comme le rayon est 5, ça touche PILLE (ou overlap epsilon).
        // On la rapproche un tout petit peu pour forcer le rebond : (12.5, 13.5).
        auto ball = builder.asCircle(5.0f).at(15.0f, 15.0f) // Départ loin
            .withVelocity(-50.0f, -50.0f) // Arrive en diagonale vers le coin
            .withMass(1.0f).withPhysicsMaterial(1.0f)
            .withCollider().build();

        // Force Position proche du coin
        registry.replace<Position>(ball, Position{12.5f, 13.5f, 0.0f});

        physics.update(registry, 0.016f);

        const auto& vBall = registry.get<Velocity>(ball);

        // VERIFICATION :
        // La balle arrivait en (-10, -10). Elle tape le coin Haut-Droit.
        // Elle doit repartir vers le Haut (Y > 0) et la Droite (X > 0).
        ExpectTrue(vBall.x > 0, "[2] Corner: Bounced Right");
        ExpectTrue(vBall.y > 0, "[2] Corner: Bounced Up");

        if(registry.all_of<Velocity>(box)) {
             const auto& vWall = registry.get<Velocity>(box);
             ExpectTrue(vWall.x == 0 && vWall.y == 0, "[1] Wall stayed static");
        }
        
        std::cout << "    -> Corner Vel Result: " << vBall.x << ", " << vBall.y << std::endl;
    }

    // ==============================================================================
    // SCÉNARIO 3 : Dedans (Deep Penetration / Spawn Inside)
    // Si une balle spawn DANS la boîte, elle doit être éjectée vers le bord le plus proche.
    // ==============================================================================
    {
        entt::registry registry;
        PhysicsSystem physics;
        physics.init();
        EntityBuilder builder(registry);

        // BOITE : 20x20 centrée en (0,0). Bords à +/- 10.
        builder.asBox(20.0f, 20.0f).at(0.0f, 0.0f)
               .withMass(0.0f).withCollider().build();

        // BALLE : Rayon 2.
        // On la place à x=8, y=0.
        // Elle est DANS la boîte.
        // Bord le plus proche : Droite (x=10). Distance = 2.
        // Bord Gauche (x=-10). Distance = 18.
        // -> Elle doit être poussée vers la DROITE.
        auto ball = builder.asCircle(2.0f).at(8.0f, 0.0f)
            .withVelocity(0.0f, 0.0f) // Immobile au début
            .withMass(1.0f).withCollider().build();

        physics.update(registry, 0.016f);

        // PhysicsSystem applique une impulsion ou modifie la position pour résoudre.
        const auto& posBall = registry.get<Position>(ball);
        
        ExpectTrue(posBall.x > 8.0f, "[3] Inside: Position ejected towards nearest edge (Right)");
    }
}

int main() {
    std::cout << "=== FULL PHYSICS TESTS ===" << std::endl;
    
    Test_Collision_Maths();
    Test_System_Circle_vs_Circle();
    Test_System_Box_vs_Box();
    Test_System_Circle_vs_Box();

    std::cout << "=== ALL DONE ===" << std::endl;
    return 0;
}