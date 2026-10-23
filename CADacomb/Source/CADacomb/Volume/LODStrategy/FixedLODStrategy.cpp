#include "FixedLODStrategy.h"

namespace CADacomb
{
    FixedLODStrategy::FixedLODStrategy(LODDecision fixedDecision)
      : decision(fixedDecision)
    {
    }

    FixedLODStrategy::~FixedLODStrategy()
    {
    }

    void FixedLODStrategy::startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution)
    {
        (void)sceneData;
        (void)volumeBounds;
        (void)vertexCapacity;
        (void)resolution;
    }

    LODDecision FixedLODStrategy::evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)
    {
        (void)bound;
        (void)currentVoxelLength;
        (void)currentLOD;
        (void)currentVertexCount;
        return decision;
    }

    void FixedLODStrategy::endFrame()
    {
    }

    LODDecision FixedLODStrategy::getDecision() const
    {
        return decision;
    }

    void FixedLODStrategy::setDecision(LODDecision newDecision)
    {
        decision = newDecision;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("FixedLODStrategy/ConstantDecision", "[FixedLODStrategy]")
    {
        FixedLODStrategy strategy(LODDecision::Refine);

        SceneData sceneData{};
        AABB3     volumeBounds(glm::dvec3(0.0), glm::dvec3(100.0));
        strategy.startFrame(sceneData, volumeBounds, 1000, 1024);

        AABB3 box(glm::dvec3(0.0), glm::dvec3(10.0));

        CHECK(strategy.evaluate(box, 1.0, 1, 500) == LODDecision::Refine);
        CHECK(strategy.evaluate(box, 1.0, 1, 1000) == LODDecision::Refine);

        strategy.setDecision(LODDecision::Coarsen);
        CHECK(strategy.evaluate(box, 1.0, 1, 1000) == LODDecision::Coarsen);

        strategy.setDecision(LODDecision::Keep);
        CHECK(strategy.evaluate(box, 1.0, 1, 500) == LODDecision::Keep);

        strategy.endFrame();
    }
}
#endif
