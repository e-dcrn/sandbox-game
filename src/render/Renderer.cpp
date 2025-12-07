#include "Renderer.hpp"
#include <iostream>
#include <vector>

#include "../common/Components.hpp"  // Pour Position
#include "../physics/Components.hpp"

// --- SHADERS (Le code qui tourne sur le GPU) ---

const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;      // Sommet du carré (-1 à 1)
    layout (location = 1) in vec2 aOffset;   // Position de la balle (x, y)
    layout (location = 2) in float aRadius;  // Rayon de la balle (optionnel si constant)

    uniform vec2 uScreenSize; // Taille de l'écran pour convertir en -1..1

    out vec2 UV; // Pour dessiner le rond

    void main() {
        UV = aPos; // On passe la coordonnée locale au pixel shader
        
        // Calcul de la position finale en pixels
        vec2 worldPos = (aPos * aRadius) + aOffset;

        // Conversion Pixels -> Normalized Device Coordinates (-1 à 1)
        // OpenGL veut des coordonnées entre -1 et 1.
        vec2 ndcPos = (worldPos / uScreenSize) * 2.0 - 1.0;
        
        // On inverse Y car OpenGL a le Y vers le haut, SDL vers le bas
        ndcPos.y = -ndcPos.y;

        gl_Position = vec4(ndcPos, 0.0, 1.0);
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    in vec2 UV;

    void main() {
        // Calcul de la distance au centre du carré
        float dist = length(UV);
        
        // Si on est hors du cercle (dist > 1), on ne dessine rien (transparent)
        // On utilise un petit "smoothstep" pour l'anti-aliasing (bords lisses)
        float alpha = 1.0 - smoothstep(0.9, 1.0, dist);

        if (alpha < 0.01) discard; // Optimisation : on jette le pixel

        // Couleur : Blanc (ou ce que tu veux)
        FragColor = vec4(0.2, 0.6, 1.0, 1.0); // Bleu clair
    }
)";

void Renderer::init() {
    // 1. Initialiser GLAD (pour charger les fonctions OpenGL)
    if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return;
    }

    compile_shaders();

    // 2. Configurer les Buffers
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &quadVBO);
    glGenBuffers(1, &instanceVBO);

    glBindVertexArray(VAO);

    // --- A. Le Carré (Modèle) ---
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    // Attribut 0 : aPos (vec2)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    // --- B. Les Instances (Positions) ---
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    // On alloue de la place pour 10 000 balles (3 floats par balle : x, y, radius)
    // Usage DYNAMIC_DRAW car ça change à chaque frame
    glBufferData(GL_ARRAY_BUFFER, 10000 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    // Attribut 1 : aOffset (vec2) -> Position
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glVertexAttribDivisor(1, 1); // IMPORTANT : Avancer de 1 par instance

    // Attribut 2 : aRadius (float)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)(2 * sizeof(float)));
    glVertexAttribDivisor(2, 1); // IMPORTANT : Avancer de 1 par instance

    glBindVertexArray(0);
}

void Renderer::render(entt::registry& registry, int width, int height) {
    // 1. Préparer les données CPU
    // On crée un vecteur plat [x, y, r, x, y, r, ...]
    static std::vector<float> instanceData;
    instanceData.clear();

    auto view = registry.view<Position, Collider>();
    // Note : ColliderType doit être CIRCLE. Si tu as des BOX, il faudra gérer à part.
    view.each([&](const auto& pos, const auto& col) {
        if (col.type == ColliderType::CIRCLE) {
            instanceData.push_back(pos.x);
            instanceData.push_back(pos.y);
            instanceData.push_back(col.radius);
        }
    });

    if (instanceData.empty()) return;

    // 2. Envoyer au GPU
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    // Orphaning (demander un nouveau buffer si besoin pour éviter d'attendre le GPU)
    glBufferData(GL_ARRAY_BUFFER, instanceData.size() * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, instanceData.size() * sizeof(float), instanceData.data());

    // 3. Dessiner
    glUseProgram(shaderProgram);
    
    // Envoyer la taille de l'écran
    glUniform2f(glGetUniformLocation(shaderProgram, "uScreenSize"), (float)width, (float)height);

    glBindVertexArray(VAO);
    // LA COMMANDE MAGIQUE : Dessine 6 sommets (1 quad) X fois le nombre de balles
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, instanceData.size() / 3);
    glBindVertexArray(0);
}

void Renderer::compile_shaders() {
    // Boilerplate de compilation OpenGL...
    GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vertexShaderSource, NULL);
    glCompileShader(vertex);
    // (Ajoute ici la vérification d'erreur de compile si tu veux être propre)

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