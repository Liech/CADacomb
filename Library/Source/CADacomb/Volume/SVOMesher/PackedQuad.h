#pragma once

#include <array>
#include <cstdint>
#include <glm/glm.hpp>

namespace Library::Volume
{
    struct PackedQuad
    {
      public:
        PackedQuad();
        explicit PackedQuad(uint64_t raw);

        static PackedQuad create(uint32_t x, uint32_t y, uint32_t z, uint32_t dir, uint32_t scaleExp, uint32_t matId = 1);
        static glm::ivec3 getNormal(uint32_t dir);

        uint32_t                  getX() const;
        uint32_t                  getY() const;
        uint32_t                  getZ() const;
        glm::uvec3                getLocalPos() const;
        uint32_t                  getDirection() const;
        uint32_t                  getScaleExp() const;
        uint32_t                  getScale() const;
        uint32_t                  getMaterial() const;
        glm::ivec3                getNormal() const;
        std::array<glm::dvec3, 4> getCorners(const glm::dvec3& chunkWorldOrigin, double voxelSize = 1.0) const;

        uint64_t packed = 0;
    };
}
