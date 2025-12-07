#include "Renderer.hpp"
#include <iostream>
#include <vector>
#include "../common/Components.hpp"
#include "../physics/Components.hpp" // <--- CRUCIAL : Pour 'Collider'

const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec2 aOffset;
    layout (location = 2) in float aRadius;
    uniform vec2 uScreenSize;
    out vec2 UV;
    void main() {
        UV = aPos;
        vec2 worldPos = (aPos * aRadius) + aOffset;
        vec2 ndcPos = (worldPos / uScreenSize) * 2.0 - 1.0;
        ndcPos.y = -ndcPos.y;
        gl_Position = vec4(ndcPos, 0.0, 1.0);
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    in vec2 UV;
    void main() {
        float dist = length(UV);
        float alpha = 1.0 - smoothstep(0.9, 1.0, dist);
        if (alpha < 0.01) discard;
        FragColor = vec4(0.2, 0.6, 1.0, 1.0);
    }
)";

void Renderer::init() {
    if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return;
    }
    compile_shaders();
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &quadVBO);
    glGenBuffers(1, &instanceVBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, 10000 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(2 * sizeof(float)));
    glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);
}

void Renderer::render(entt::registry& registry, int width, int height) {
    static std::vector<float> instanceData;
    instanceData.clear();
    // OPTIMISATION : Reserve mémoire
    // instanceData.reserve(3000 * 3);

    auto view = registry.view<Position, Collider>();
    view.each([&](const auto& pos, const auto& col) {
        if (col.type == ColliderType::CIRCLE) {
            instanceData.push_back(pos.x);
            instanceData.push_back(pos.y);
            instanceData.push_back(col.radius);
        }
    });

    if (instanceData.empty()) return;

    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instanceData.size() * sizeof(float), instanceData.data());

    glUseProgram(shaderProgram);
    glUniform2f(glGetUniformLocation(shaderProgram, "uScreenSize"), (float)width, (float)height);

    glBindVertexArray(VAO);
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, instanceData.size() / 3);
    glBindVertexArray(0);
}

void Renderer::compile_shaders() {
    GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vertexShaderSource, NULL);
    glCompileShader(vertex);
    GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragment);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertex);
    glAttachShader(shaderProgram, fragment);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

void Renderer::cleanup() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &quadVBO);
    glDeleteBuffers(1, &instanceVBO);
    glDeleteProgram(shaderProgram);
}