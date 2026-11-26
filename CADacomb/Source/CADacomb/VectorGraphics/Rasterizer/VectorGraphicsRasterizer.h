#pragma once

#include "CADacomb/Texture/Texture.h"
#include "CADacomb/VectorGraphics/Data/VectorFillRule.h"
#include "CADacomb/VectorGraphics/Data/VectorGraphics.h"

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::VectorGraphics
{
    class VectorElement;
    class VectorPath;

    // Software scanline rasterizer converting a VectorGraphics document into an RGBA8 Texture.
    class VectorGraphicsRasterizer
    {
      public:
        VectorGraphicsRasterizer();
        virtual ~VectorGraphicsRasterizer();

        static CADacomb::Texture::Texture rasterize(const VectorGraphics& document, uint32_t width, uint32_t height, const glm::u8vec4& backgroundColor = glm::u8vec4(0, 0, 0, 0));

      private:
        static void renderElement(const VectorElement& element, const glm::dmat3& parentTransform, CADacomb::Texture::Texture& target);
        static void renderPath(const VectorPath& path, const glm::dmat3& transform, CADacomb::Texture::Texture& target);
        static void rasterizeFill(const std::vector<std::vector<glm::dvec2>>& polylines, VectorFillRule fillRule, const glm::u8vec4& color, CADacomb::Texture::Texture& target);
        static void rasterizeStroke(const std::vector<std::vector<glm::dvec2>>& polylines,
                                    const std::vector<bool>&                    closedFlags,
                                    double                                      strokeWidth,
                                    const glm::u8vec4&                          color,
                                    CADacomb::Texture::Texture&                 target);
        static void blendPixel(CADacomb::Texture::Texture& target, uint32_t x, uint32_t y, const glm::u8vec4& color, double coverage);
    };
}
