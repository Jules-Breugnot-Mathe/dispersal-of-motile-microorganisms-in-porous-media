#ifndef DISK_RENDERER_HPP
#define DISK_RENDERER_HPP

#include "rendering/Renderer.hpp"
#include "simulation/Disk.hpp"

class DiskRenderer : public Renderer {
public:
    DiskRenderer(const Disk& disk, int segments = 32);
    void draw(const Shader& shader) const override;
};

#endif
