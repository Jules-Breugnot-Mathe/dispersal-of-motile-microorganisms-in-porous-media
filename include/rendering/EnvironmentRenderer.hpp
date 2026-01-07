#ifndef ENVIRONMENT_RENDERER_HPP
#define ENVIRONMENT_RENDERER_HPP

#include <vector>
#include <memory>

#include "Renderer.hpp"
#include "Environment.hpp"

class EnvironmentRenderer {
private:
    std::vector<std::unique_ptr<Renderer>> renderers;

public:
    explicit EnvironmentRenderer(const Environment& env);
    void draw(const Shader& shader) const;
};

#endif
