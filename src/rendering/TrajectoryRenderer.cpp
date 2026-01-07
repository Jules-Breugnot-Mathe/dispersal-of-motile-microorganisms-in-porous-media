#include "TrajectoryRenderer.hpp"
#include <glad/glad.h>

TrajectoryRenderer::TrajectoryRenderer()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(
        0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0
    );
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void TrajectoryRenderer::addPoint(const Point& p)
{
    vertices.push_back(static_cast<float>(p.getx()));
    vertices.push_back(static_cast<float>(p.gety()));

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(
        GL_ARRAY_BUFFER,
        vertices.size() * sizeof(float),
        vertices.data(),
        GL_DYNAMIC_DRAW
    );
}

void TrajectoryRenderer::draw(const Shader& shader) const
{
    shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINE_STRIP, 0, vertices.size() / 2);
    glBindVertexArray(0);
}

void TrajectoryRenderer::clearTrajectory(){
    this->vertices.clear();
}