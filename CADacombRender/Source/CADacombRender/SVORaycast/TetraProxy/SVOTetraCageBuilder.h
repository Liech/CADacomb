#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacombRender/SVORaycast/TetraProxy/GPUTetraVertex.h"
#include "CADacombRender/SVORaycast/TetraProxy/GPUTetrahedron.h"
#include "CADacombRender/SVORaycast/TetraProxy/TetraSurfaceVertex.h"

#include <cstdint>
#include <vector>

namespace CADacomb::Render
{
    // Builds a conforming 5-tetrahedra-per-cell deformation cage covering occupied regions of an SVO.
    class SVOTetraCageBuilder
    {
      public:
        SVOTetraCageBuilder();
        virtual ~SVOTetraCageBuilder();

        static void build(const CADacomb::SVO<bool>&       svo,
                          const CADacomb::AABB3&           volumeBounds,
                          uint32_t                         gridResolution,
                          std::vector<GPUTetraVertex>&     outVertices,
                          std::vector<GPUTetrahedron>&     outTetrahedra,
                          std::vector<TetraSurfaceVertex>& outSurfaceVertices);

      private:
        static bool     isCellOccupied(const CADacomb::SVO<bool>& svo, uint32_t cx, uint32_t cy, uint32_t cz, uint32_t cellSize);
        static uint32_t clampPowerOfTwo(uint32_t value, uint32_t maxValue);
        static int      log2Uint(uint32_t value);
        static uint64_t makeFaceKey(uint32_t a, uint32_t b, uint32_t c);
        static void     orientTetrahedron(const std::vector<GPUTetraVertex>& vertices, glm::uvec4& indices);
    };
}
