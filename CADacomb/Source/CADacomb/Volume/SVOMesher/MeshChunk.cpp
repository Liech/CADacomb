#include "MeshChunk.h"
#include <algorithm>

namespace CADacomb
{
    MeshChunk::MeshChunk() {}

    MeshChunk::~MeshChunk() {}

    uint32_t MeshChunk::getMinCellSize() const
    {
        if (maxResolution == 0 || size <= maxResolution)
        {
            return 1u;
        }
        return size / maxResolution;
    }

    uint32_t MeshChunk::getCellSize() const
    {
        uint32_t boundMin = getMinCellSize();
        uint32_t lodCell  = 1u << std::clamp(lodLevel, 0, 16);
        uint32_t maxCell  = std::max(1u, size);
        return std::min(maxCell, std::max(boundMin, lodCell));
    }

    uint32_t MeshChunk::getEffectiveGridResolution() const
    {
        return size / getCellSize();
    }

    uint32_t MeshChunk::getSecondaryMinCellSize() const
    {
        if (secondaryMaxResolution == 0 || size <= secondaryMaxResolution)
        {
            return 1u;
        }
        return size / secondaryMaxResolution;
    }

    uint32_t MeshChunk::getSecondaryCellSize() const
    {
        uint32_t boundMin = getSecondaryMinCellSize();
        uint32_t lodCell  = 1u << std::clamp(secondaryLodLevel, 0, 16);
        uint32_t maxCell  = std::max(1u, size);
        return std::min(maxCell, std::max(boundMin, lodCell));
    }

    MeshChunk MeshChunk::createBorder(const glm::ivec3& negChunkMinBound, uint32_t chunkSize, uint8_t axis, bool hasNeg, int negLod, uint32_t negMaxRes, bool hasPos, int posLod, uint32_t posMaxRes)
    {
        MeshChunk border;
        border.minBound               = negChunkMinBound;
        border.size                   = chunkSize;
        border.isBorder               = true;
        border.borderAxis             = axis;
        border.hasNegativeChunk       = hasNeg;
        border.lodLevel               = negLod;
        border.maxResolution          = negMaxRes;
        border.hasPositiveChunk       = hasPos;
        border.secondaryLodLevel      = posLod;
        border.secondaryMaxResolution = posMaxRes;
        return border;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("SVOMesher/MeshChunkCellSizeResolution", "[SVOMesher][MeshChunk]")
    {
        MeshChunk chunk;
        chunk.size          = 64;
        chunk.maxResolution = 16;
        chunk.lodLevel      = 0;

        CHECK(chunk.getMinCellSize() == 4);
        CHECK(chunk.getCellSize() == 4);
        CHECK(chunk.getEffectiveGridResolution() == 16);

        chunk.lodLevel = 3;
        CHECK(chunk.getCellSize() == 8);
        CHECK(chunk.getEffectiveGridResolution() == 8);

        chunk.lodLevel = 10;
        CHECK(chunk.getCellSize() == 64);
        CHECK(chunk.getEffectiveGridResolution() == 1);
    }

    TEST_CASE("SVOMesher/MeshChunkBorderCreation", "[SVOMesher][MeshChunk]")
    {
        MeshChunk border = MeshChunk::createBorder(glm::ivec3(0, 0, 0), 32, 0, true, 0, 32, true, 2, 16);

        CHECK(border.isBorder);
        CHECK(border.borderAxis == 0);
        CHECK(border.hasNegativeChunk);
        CHECK(border.hasPositiveChunk);
        CHECK(border.getCellSize() == 1);
        CHECK(border.getSecondaryCellSize() == 4);
    }
}
#endif
