#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Texture
{
    // Per-face-corner UV coordinates (canonical top-left origin) decoupled from Triangulation geometry.
    class TriangulationUVs
    {
      public:
        TriangulationUVs();
        TriangulationUVs(size_t cornerCount);
        virtual ~TriangulationUVs();

        size_t getCornerCount() const;
        bool   isEmpty() const;
        void   resize(size_t cornerCount);

        const glm::dvec2&              getUV(size_t cornerIndex) const;
        glm::dvec2&                    getUV(size_t cornerIndex);
        void                           setUV(size_t cornerIndex, const glm::dvec2& uv);
        const std::vector<glm::dvec2>& getAllUVs() const;

        void flipV();

      private:
        std::vector<glm::dvec2> cornerUVs;
    };
}
