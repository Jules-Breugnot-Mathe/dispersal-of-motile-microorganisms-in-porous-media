#ifndef MOBILE_RENDERER_HPP
#define MOBILE_RENDERER_HPP

#include "rendering/Renderer.hpp"
#include "rendering/Shader.hpp"
#include "simulation/Mobile.hpp"

class MobileRenderer : public Renderer {
public:
    explicit MobileRenderer(const Mobile& mobile);
    ~MobileRenderer();

    void draw(const Shader& shader) const override;

private:
    const Mobile& mobile;
};

#endif
