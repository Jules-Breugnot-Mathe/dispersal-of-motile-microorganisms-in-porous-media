#ifndef INFINITE_ENVIRONMENT_RENDERER_HPP
#define INFINITE_ENVIRONMENT_RENDERER_HPP

#include "Environment.hpp"
#include "rendering/DiskRenderer.hpp"
#include "rendering/RectangleRenderer.hpp"
#include "rendering/Shader.hpp"

#include <vector>
#include <memory>

class InfiniteEnvironmentRenderer {
private:
    const Environment& env;

    std::vector<std::unique_ptr<Renderer>> solidRenderers;

    double domainRadius;
    double tileSize;

public:
    explicit InfiniteEnvironmentRenderer(const Environment& env);

    void draw(const Shader& shader, float z_cam) const;
};

#endif
