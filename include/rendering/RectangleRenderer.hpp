#ifndef RECTANGLE_RENDERER_HPP
#define RECTANGLE_RENDERER_HPP

#include "rendering/Renderer.hpp"
#include "rendering/Shader.hpp"
#include "simulation/Rectangle.hpp"

class RectangleRenderer : public Renderer {
public:
    RectangleRenderer(const Rectangle& rect);
    ~RectangleRenderer();

    void draw(const Shader& shader) const override;
};

#endif
