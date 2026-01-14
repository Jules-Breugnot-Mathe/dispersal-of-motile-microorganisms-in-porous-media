#include "rendering/InfiniteEnvironmentRenderer.hpp"
#include <glad/glad.h>
#include <cmath>

InfiniteEnvironmentRenderer::InfiniteEnvironmentRenderer(const Environment& env)
    : env(env)
{
    domainRadius = env.getDomain().getradius();
    tileSize = 2.0 * domainRadius;

    // Création des renderers à partir du domaine fondamental
    for (const auto& solid : env.get_Solid_vector()) {
        if (auto d = dynamic_cast<Disk*>(solid.get())) {
            solidRenderers.push_back(
                std::make_unique<DiskRenderer>(*d)
            );
        }
        else if (auto r = dynamic_cast<Rectangle*>(solid.get())) {
            solidRenderers.push_back(
                std::make_unique<RectangleRenderer>(*r)
            );
        }
    }
}

void InfiniteEnvironmentRenderer::draw(const Shader& shader, float z_cam) const
{
    shader.use();

    // Distance algébrique caméra-plan Oxy 
    float visibleRadius = z_cam * 0.8f;

    int N = static_cast<int>(std::ceil(visibleRadius / tileSize)) + 1;

    int locOffset = glGetUniformLocation(shader.ID, "modelOffset");

    for (int i = -N; i <= N; ++i) {
        for (int j = -N; j <= N; ++j) {

            float ox = static_cast<float>(i * tileSize);
            float oy = static_cast<float>(j * tileSize);

            glUniform2f(locOffset, ox, oy);

            for (const auto& r : solidRenderers) {
                r->draw(shader);
            }
        }
    }
}
