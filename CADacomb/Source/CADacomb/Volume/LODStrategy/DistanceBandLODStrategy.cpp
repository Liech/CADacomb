#include "DistanceBandLODStrategy.h"

namespace CADacomb
{
    DistanceBandLODStrategy::DistanceBandLODStrategy(double nearDistance, double farDistance)
      : nearDistance(nearDistance)
      , farDistance(farDistance)
    {
    }

    DistanceBandLODStrategy::~DistanceBandLODStrategy()
    {
    }

    void DistanceBandLODStrategy::startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution)
    {
        (void)volumeBounds;
        (void)vertexCapacity;
        (void)resolution;
        cameraPos = glm::dvec3(sceneData.cameraPos);
    }

    LODDecision DistanceBandLODStrategy::evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)
    {
        (void)currentVoxelLength;
        (void)currentLOD;
        (void)currentVertexCount;

        double distance = bound.distanceTo(cameraPos);

        if (distance < nearDistance)
        {
            return LODDecision::Refine;
        }

        if (distance > farDistance)
        {
            return LODDecision::Coarsen;
        }

        return LODDecision::Keep;
    }

    void DistanceBandLODStrategy::endFrame()
    {
    }

    double DistanceBandLODStrategy::getNearDistance() const
    {
        return nearDistance;
    }

    double DistanceBandLODStrategy::getFarDistance() const
    {
        return farDistance;
    }

    void DistanceBandLODStrategy::setNearDistance(double nearDist)
    {
        nearDistance = nearDist;
    }

    void DistanceBandLODStrategy::setFarDistance(double farDist)
    {
        farDistance = farDist;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("DistanceBandLODStrategy/DecisionBands", "[DistanceBandLODStrategy]")
    {
        DistanceBandLODStrategy strategy(10.0, 50.0);

        SceneData sceneData{};
        sceneData.cameraPos = glm::dvec4(0.0, 0.0, 0.0, 1.0);

        AABB3 volumeBounds(glm::dvec3(0.0), glm::dvec3(100.0));
        strategy.startFrame(sceneData, volumeBounds, 10000, 1024);

        AABB3 closeBox(glm::dvec3(4.0), glm::dvec3(6.0));
        AABB3 midBox(glm::dvec3(20.0), glm::dvec3(25.0));
        AABB3 farBox(glm::dvec3(80.0), glm::dvec3(90.0));

        CHECK(strategy.evaluate(closeBox, 1.0, 1, 100) == LODDecision::Refine);
        CHECK(strategy.evaluate(midBox, 1.0, 1, 100) == LODDecision::Keep);
        CHECK(strategy.evaluate(farBox, 1.0, 1, 100) == LODDecision::Coarsen);

        strategy.endFrame();
    }
}
#endif
