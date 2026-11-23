#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb::Render
{
    // Background meshing job descriptor for one SVOLODMesh chunk.
    struct SVOMeshJob
    {
      public:
        SVOMeshJob();
        virtual ~SVOMeshJob();

        size_t             chunkIndex = 0;
        uint64_t           generation = 0;
        glm::ivec3         minBound{ 0 };
        uint32_t           size     = 32;
        int                lodLevel = 0;
        std::array<int, 6> neighborLODs{ 0, 0, 0, 0, 0, 0 };
        glm::vec3          color{ 0.8f, 0.8f, 0.8f };
        bool               computeAO = false;
        double             distSq    = 0.0;
    };
}
