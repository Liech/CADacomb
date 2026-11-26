#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacombRender/SVORaycast/TetraProxy/GPUTetraVertex.h"

#include <cstdint>
#include <vector>

namespace CADacomb::Render
{
    // Procedural sine-wave bending deformer for testing deformable tetrahedron SVO raycasting.
    class TetraSineDeformer
    {
      public:
        TetraSineDeformer();
        virtual ~TetraSineDeformer();

        static void deform(std::vector<GPUTetraVertex>& vertices,
                           const CADacomb::AABB3&       volumeBounds,
                           uint32_t                     svoDimension,
                           float                        timeSeconds,
                           float                        amplitudeFactor = 0.08f);
    };
}
