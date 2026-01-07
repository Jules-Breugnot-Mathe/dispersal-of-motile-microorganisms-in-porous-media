#include "rendering/RectangleRenderer.hpp"
#include <glad/glad.h>

RectangleRenderer::RectangleRenderer(const Rectangle& rect)
{
    // Récupération des sommets
    float vertices[] = {
        // triangle 1
        (float)rect.getVertex(0).getx(), (float)rect.getVertex(0).gety(),
        (float)rect.getVertex(1).getx(), (float)rect.getVertex(1).gety(),
        (float)rect.getVertex(2).getx(), (float)rect.getVertex(2).gety(),

        // triangle 2
        (float)rect.getVertex(0).getx(), (float)rect.getVertex(0).gety(),
        (float)rect.getVertex(2).getx(), (float)rect.getVertex(2).gety(),
        (float)rect.getVertex(3).getx(), (float)rect.getVertex(3).gety()
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(
        0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0
    );
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

RectangleRenderer::~RectangleRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void RectangleRenderer::draw(const Shader& shader) const
{
    shader.use();
    shader.setVec3("color", 0.3f, 0.3f, 1.0f);

    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}
