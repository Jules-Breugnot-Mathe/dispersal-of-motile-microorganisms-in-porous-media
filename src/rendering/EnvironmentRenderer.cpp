#include "rendering/EnvironmentRenderer.hpp"

#include "Disk.hpp"
#include "Rectangle.hpp"
#include "DiskRenderer.hpp"
#include "RectangleRenderer.hpp"

EnvironmentRenderer::EnvironmentRenderer(const Environment& env)
{
    const auto& solids = env.get_Solid_vector();

    for (const auto& s : solids)
    {
        if (const Disk* d = dynamic_cast<const Disk*>(s.get()))
        {
            renderers.push_back(
                std::make_unique<DiskRenderer>(*d)
            );
        }
        else if (const Rectangle* r = dynamic_cast<const Rectangle*>(s.get()))
        {
            renderers.push_back(
                std::make_unique<RectangleRenderer>(*r)
            );
        }
    }
}

void EnvironmentRenderer::draw(const Shader& shader) const
{
    for (const auto& r : renderers)
    {
        r->draw(shader); // ✔ signature correcte
    }
}
