#include "CapacityTargetLODStrategy.h"
#include <algorithm>
#include <cmath>

namespace CADacomb
{
    CapacityTargetLODStrategy::CapacityTargetLODStrategy(double targetFillRatio, double errorMargin, double initialLogScale)
      : targetFillRatio(targetFillRatio)
      , errorMargin(errorMargin)
      , logScale(initialLogScale)
    {
    }

    CapacityTargetLODStrategy::~CapacityTargetLODStrategy()
    {
    }

    void CapacityTargetLODStrategy::startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t newVertexCapacity, size_t resolution)
    {
        cameraPos      = glm::dvec3(sceneData.cameraPos);
        vertexCapacity = newVertexCapacity;

        double screenHeight     = static_cast<double>(sceneData.resolution.y);
        double cotFovHalf       = std::abs(static_cast<double>(sceneData.projectionMatrix[1][1]));
        double projectionFactor = 0.5 * cotFovHalf * screenHeight;

        double leafVoxelLength = volumeBounds.getSize().x / static_cast<double>(std::max(size_t(1), resolution));
        refDistance            = std::max(1e-4, leafVoxelLength * projectionFactor);
    }

    LODDecision CapacityTargetLODStrategy::evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)
    {
        (void)currentVoxelLength;
        lastVertexCount = currentVertexCount;

        double distance  = std::max(0.0, bound.distanceTo(cameraPos));
        double normDist  = distance / refDistance;
        int    targetLOD = static_cast<int>(logScale * std::log2(1.0 + normDist));

        if (currentLOD > targetLOD)
        {
            double targetCount = targetFillRatio * static_cast<double>(vertexCapacity);
            double upperLimit  = targetCount * (1.0 + errorMargin);
            if (currentVertexCount >= static_cast<size_t>(upperLimit) || currentVertexCount >= vertexCapacity)
            {
                return LODDecision::Keep;
            }
            return LODDecision::Refine;
        }

        if (currentLOD < targetLOD)
        {
            return LODDecision::Coarsen;
        }

        return LODDecision::Keep;
    }

    void CapacityTargetLODStrategy::endFrame()
    {
        double targetCount = targetFillRatio * static_cast<double>(vertexCapacity);
        double upperLimit  = targetCount * (1.0 + errorMargin);
        double lowerLimit  = targetCount * (1.0 - errorMargin);
        double current     = static_cast<double>(lastVertexCount);

        if (current > 0.0 && (current > upperLimit || current < lowerLimit))
        {
            double ratio          = std::sqrt(current / targetCount);
            double targetLogScale = logScale * ratio;
            logScale              = logScale + 0.5 * (targetLogScale - logScale);
        }

        logScale = std::clamp(logScale, 0.1, 500.0);
    }

    double CapacityTargetLODStrategy::getTargetFillRatio() const
    {
        return targetFillRatio;
    }

    void CapacityTargetLODStrategy::setTargetFillRatio(double ratio)
    {
        targetFillRatio = ratio;
    }

    double CapacityTargetLODStrategy::getErrorMargin() const
    {
        return errorMargin;
    }

    void CapacityTargetLODStrategy::setErrorMargin(double margin)
    {
        errorMargin = margin;
    }

    double CapacityTargetLODStrategy::getLogScale() const
    {
        return logScale;
    }

    void CapacityTargetLODStrategy::setLogScale(double scale)
    {
        logScale = scale;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("CapacityTargetLODStrategy/LogarithmicFalloffAndPerspectiveSteering", "[CapacityTargetLODStrategy]")
    {
        CapacityTargetLODStrategy strategy(0.9, 0.05, 1.0);

        SceneData sceneData{};
        sceneData.resolution             = glm::dvec2(1920.0, 1080.0);
        sceneData.projectionMatrix       = glm::dmat4(1.0);
        sceneData.projectionMatrix[1][1] = 1.7320508;
        sceneData.cameraPos              = glm::dvec4(0.0, 0.0, 0.0, 1.0);

        AABB3 volumeBounds(glm::dvec3(0.0), glm::dvec3(100.0));
        strategy.startFrame(sceneData, volumeBounds, 1000, 1024);

        // Near box (distance ~ 0): targetLOD = 0
        AABB3 closeBox(glm::dvec3(0.1, 0.0, 0.0), glm::dvec3(0.2, 0.1, 0.1));
        CHECK(strategy.evaluate(closeBox, 0.1, 2, 500) == LODDecision::Refine);

        // Even when current is at 900 (targetCount), within upperLimit (945) chunks can still refine if needed
        CHECK(strategy.evaluate(closeBox, 0.1, 2, 900) == LODDecision::Refine);

        // When exceeding upperLimit (945), refining is blocked
        CHECK(strategy.evaluate(closeBox, 0.1, 2, 950) == LODDecision::Keep);

        // Far box (distance ~ 500): targetLOD is coarse
        AABB3 farBox(glm::dvec3(500.0, 0.0, 0.0), glm::dvec3(501.0, 1.0, 1.0));
        CHECK(strategy.evaluate(farBox, 0.1, 1, 500) == LODDecision::Coarsen);

        // Steering test:
        // Above upperLimit: logScale increases
        strategy.evaluate(farBox, 0.1, 1, 980);
        double prevLogScale = strategy.getLogScale();
        strategy.endFrame();
        CHECK(strategy.getLogScale() > prevLogScale);

        // Below lowerLimit: logScale decreases
        strategy.startFrame(sceneData, volumeBounds, 1000, 1024);
        strategy.evaluate(farBox, 0.1, 1, 500);
        prevLogScale = strategy.getLogScale();
        strategy.endFrame();
        CHECK(strategy.getLogScale() < prevLogScale);
    }
}
#endif
