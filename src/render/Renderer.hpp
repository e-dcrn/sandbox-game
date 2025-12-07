#pragma once
#include <vector>
#include <entt/entt.hpp>
#include "../common/Components.hpp" // Pour accéder à Position

// Inclus GLAD avant tout autre truc OpenGL/SDL
#include <glad/glad.h> 
#include <SDL2/SDL.h>

class Renderer {
public:
    void init();
    void render(entt::registry& registry, int width, int height);
    void cleanup();

private:
    GLuint shaderProgram;
    GLuint VAO, quadVBO, instanceVBO;
    
    // Le modèle d'un cercle (un simple carré de 2x2 centré en 0,0)
    // On va le sculpter en cercle dans le pixel shader
    const float quadVertices[12] = {
        -1.0f,  1.0f, // Haut Gauche
        -1.0f, -1.0f, // Bas Gauche
         1.0f, -1.0f, // Bas Droite
        
        -1.0f,  1.0f, // Haut Gauche
         1.0f, -1.0f, // Bas Droite
         1.0f,  1.0f  // Haut Droite
    };

    void compile_shaders();
};