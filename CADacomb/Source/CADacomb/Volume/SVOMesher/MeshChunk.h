#pragma once

#include "PackedQuad.h"
#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    struct MeshChunk
    {
      public:
        MeshChunk();
        virtual ~MeshChunk();

        uint32_t getMinCellSize() const;
        uint32_t getCellSize() const;
        uint32_t getEffectiveGridResolution() const;

        uint32_t getSecondaryMinCellSize() const;
        uint32_t getSecondaryCellSize() const;

        static MeshChunk createBorder(const glm::ivec3& negChunkMinBound, uint32_t chunkSize, uint8_t axis, bool hasNeg, int negLod, uint32_t negMaxRes, bool hasPos, int posLod, uint32_t posMaxRes);

        std::vector<PackedQuad> quads;
        size_t                  evaluatedCellCount = 0;
        glm::ivec3              minBound{ 0 };
        uint32_t                size                   = 32;
        int                     lodLevel               = 0;
        uint32_t                maxResolution          = 1024;
        int                     secondaryLodLevel      = 0;
        uint32_t                secondaryMaxResolution = 1024;
        uint8_t                 borderAxis             = 0;
        bool                    isBorder               = false;
        bool                    hasNegativeChunk       = true;
        bool                    hasPositiveChunk       = true;
    };
}
