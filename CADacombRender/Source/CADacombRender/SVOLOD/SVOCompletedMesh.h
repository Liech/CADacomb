#pragma once

#include "Vulcant/Toolkit/MeshRenderer.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace CADacomb::Render
{
    // Completed background mesh payload ready for GPU upload in SVOLODMesh.
    struct SVOCompletedMesh
    {
      public:
        SVOCompletedMesh();
        virtual ~SVOCompletedMesh();

        size_t                                    chunkIndex = 0;
        uint64_t                                  generation = 0;
        int                                       lodLevel   = 0;
        std::vector<Vulcant::Toolkit::MeshVertex> vertices;
    };
}
