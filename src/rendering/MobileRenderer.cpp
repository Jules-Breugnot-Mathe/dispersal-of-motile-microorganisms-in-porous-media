#include "rendering/MobileRenderer.hpp"
#include <glad/glad.h>

MobileRenderer::MobileRenderer(const Mobile& m)
    : mobile(m)
{
    // Un simple point (sera repositionné dynamiquement)
    float vertex[] = {
        0.0f, 0.0f
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(vertex),
        vertex,
        GL_DYNAMIC_DRAW
    );

    glVertexAttribPointer(
        0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0
    );
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

MobileRenderer::~MobileRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void MobileRenderer::draw(const Shader& shader) const
{
    // Récupération des coordonnées réelles
    Point p = mobile.getFreeCoord();

    float vertex[] = {
        static_cast<float>(p.getx()),
        static_cast<float>(p.gety())
    };

    // Mise à jour dynamique du VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertex), vertex);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    shader.use();
    shader.setVec3("color", 0.0f, 1.0f, 0.0f); // vert

    glPointSize(8.0f); // point bien visible
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, 1);
}
