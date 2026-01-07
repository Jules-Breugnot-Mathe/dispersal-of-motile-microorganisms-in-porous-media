#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "rendering/Shader.hpp"

class Renderer {
protected:
    unsigned int VAO, VBO;
public:
    virtual void draw(const Shader& shader) const = 0;
    virtual ~Renderer();
};

#endif
