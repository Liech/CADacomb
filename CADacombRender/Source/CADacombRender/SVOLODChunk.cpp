#include "CADacombRender/SVOLODChunk.h"

namespace CADacomb::Render
{
    SVOLODChunk::SVOLODChunk() {}

    SVOLODChunk::SVOLODChunk(const glm::ivec3& minBound, uint32_t size, const CADacomb::AABB3& worldBounds, int initialLOD)
      : worldBounds(worldBounds)
      , minBound(minBound)
      , size(size)
      , lodLevel(initialLOD)
      , dirty(true)
    {
    }

    SVOLODChunk::~SVOLODChunk() {}

    const CADacomb::AABB3& SVOLODChunk::getWorldBounds() const
    {
        return worldBounds;
    }

    const glm::ivec3& SVOLODChunk::getMinBound() const
    {
        return minBound;
    }

    uint32_t SVOLODChunk::getSize() const
    {
        return size;
    }

    int SVOLODChunk::getLOD() const
    {
        return lodLevel;
    }

    void SVOLODChunk::setLOD(int lod)
    {
        lodLevel = lod;
    }

    size_t SVOLODChunk::getPackageId() const
    {
        return packageId;
    }

    void SVOLODChunk::setPackageId(size_t id)
    {
        packageId  = id;
        packageSet = true;
    }

    bool SVOLODChunk::hasPackage() const
    {
        return packageSet;
    }

    void SVOLODChunk::clearPackage()
    {
        packageId  = 0;
        packageSet = false;
    }

    bool SVOLODChunk::isDirty() const
    {
        return dirty;
    }

    void SVOLODChunk::setDirty(bool newDirty)
    {
        dirty = newDirty;
    }

    double SVOLODChunk::getVoxelLength(double leafVoxelLength) const
    {
        return leafVoxelLength * static_cast<double>(1u << lodLevel);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb::Render
{
    TEST_CASE("SVOLODChunk/LifecycleAndPackage", "[SVOLODChunk]")
    {
        CADacomb::AABB3 bounds(glm::dvec3(0.0), glm::dvec3(32.0));
        SVOLODChunk     chunk(glm::ivec3(0), 32, bounds, 2);

        CHECK(chunk.getMinBound() == glm::ivec3(0));
        CHECK(chunk.getSize() == 32);
        CHECK(chunk.getLOD() == 2);
        CHECK(chunk.isDirty());
        CHECK(!chunk.hasPackage());

        chunk.setPackageId(42);
        CHECK(chunk.hasPackage());
        CHECK(chunk.getPackageId() == 42);

        chunk.clearPackage();
        CHECK(!chunk.hasPackage());
        CHECK(chunk.getPackageId() == 0);

        chunk.setLOD(0);
        CHECK(chunk.getLOD() == 0);
        CHECK(chunk.getVoxelLength(0.5) == 0.5);

        chunk.setLOD(3);
        CHECK(chunk.getVoxelLength(0.5) == 4.0);
    }
}
#endif
