#include "ScreenSpaceLODStrategy.h"
#include <algorithm>
#include <cmath>

namespace CADacomb
{
    ScreenSpaceLODStrategy::ScreenSpaceLODStrategy(double minPixelSize, double maxPixelSize)
      : minPixelSize(minPixelSize)
      , maxPixelSize(maxPixelSize)
    {
    }

    ScreenSpaceLODStrategy::~ScreenSpaceLODStrategy()
    {
    }

    void ScreenSpaceLODStrategy::startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution)
    {
        (void)volumeBounds;
        (void)vertexCapacity;
        (void)resolution;
        cameraPos = glm::dvec3(sceneData.cameraPos);

        double screenHeight = static_cast<double>(sceneData.resolution.y);
        double cotFovHalf   = std::abs(static_cast<double>(sceneData.projectionMatrix[1][1]));
        projectionFactor    = 0.5 * cotFovHalf * screenHeight;
    }

    LODDecision ScreenSpaceLODStrategy::evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)
    {
        (void)currentLOD;
        (void)currentVertexCount;

        double distance        = std::max(1e-4, bound.distanceTo(cameraPos));
        double projectedPixels = (currentVoxelLength * projectionFactor) / distance;

        if (projectedPixels > maxPixelSize)
        {
            return LODDecision::Refine;
        }

        if (projectedPixels < minPixelSize)
        {
            return LODDecision::Coarsen;
        }

        return LODDecision::Keep;
    }

    void ScreenSpaceLODStrategy::endFrame()
    {
    }

    double ScreenSpaceLODStrategy::getMinPixelSize() const
    {
        return minPixelSize;
    }

    double ScreenSpaceLODStrategy::getMaxPixelSize() const
    {
        return maxPixelSize;
    }

    void ScreenSpaceLODStrategy::setMinPixelSize(double minPixels)
    {
        minPixelSize = minPixels;
    }

    void ScreenSpaceLODStrategy::setMaxPixelSize(double maxPixels)
    {
        maxPixelSize = maxPixels;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("ScreenSpaceLODStrategy/DecisionBands", "[ScreenSpaceLODStrategy]")
    {
        ScreenSpaceLODStrategy strategy(2.0, 4.0);

        SceneData sceneData{};
        sceneData.resolution             = glm::dvec2(1920.0, 1080.0);
        sceneData.projectionMatrix       = glm::dmat4(1.0);
        sceneData.projectionMatrix[1][1] = 1.7320508;
        sceneData.cameraPos              = glm::dvec4(0.0, 0.0, 0.0, 1.0);

        AABB3 volumeBounds(glm::dvec3(0.0), glm::dvec3(100.0));
        strategy.startFrame(sceneData, volumeBounds, 10000, 1024);

        AABB3 closeBox(glm::dvec3(9.0, -1.0, -1.0), glm::dvec3(11.0, 1.0, 1.0));
        AABB3 midBox(glm::dvec3(29.0, -1.0, -1.0), glm::dvec3(31.0, 1.0, 1.0));
        AABB3 farBox(glm::dvec3(99.0, -1.0, -1.0), glm::dvec3(101.0, 1.0, 1.0));

        double voxelLength = 0.1;

        CHECK(strategy.evaluate(closeBox, voxelLength, 1, 100) == LODDecision::Refine);
        CHECK(strategy.evaluate(midBox, voxelLength, 1, 100) == LODDecision::Keep);
        CHECK(strategy.evaluate(farBox, voxelLength, 1, 100) == LODDecision::Coarsen);

        strategy.endFrame();
    }
}
#endif
