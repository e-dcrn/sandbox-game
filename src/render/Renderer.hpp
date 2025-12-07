#pragma once
#include <vector>
#include <entt/entt.hpp>
#include "../common/Components.hpp" 
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
    
    const float quadVertices[12] = {
        -1.0f,  1.0f, -1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f 
    };

    void compile_shaders();
};