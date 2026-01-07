#include "rendering/DiskRenderer.hpp"
#include <vector>
#include <cmath>

# define M_PI           3.14159265358979323846  /* pi */

using namespace std;

DiskRenderer::DiskRenderer(const Disk& disk, int segments) {
    std::vector<float> vertices;

    float cx = disk.getCenter().getx();
    float cy = disk.getCenter().gety();
    float r  = disk.getRadius();

    for (int i = 0; i < segments; ++i) {
        float a = 2.f * M_PI * i / segments;
        vertices.push_back(cx + r * cos(a));
        vertices.push_back(cy + r * sin(a));
    }

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float),
                 vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void DiskRenderer::draw(const Shader& shader) const {
    shader.use();
    shader.setVec3("color", 1.f, 0.f, 0.f);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 32);
}
