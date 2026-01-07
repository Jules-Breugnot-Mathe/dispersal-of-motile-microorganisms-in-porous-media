#ifndef TRAJECTORY_RENDERER_HPP
#define TRAJECTORY_RENDERER_HPP

#include "Renderer.hpp"
#include "Point.hpp"
#include <vector>

class TrajectoryRenderer : public Renderer {
private:
    std::vector<float> vertices; // x0 y0 x1 y1 ...

public:
    TrajectoryRenderer();
    ~TrajectoryRenderer() = default;

    void addPoint(const Point& p);
    void clearTrajectory();
    void draw(const Shader& shader) const override;
};

#endif
