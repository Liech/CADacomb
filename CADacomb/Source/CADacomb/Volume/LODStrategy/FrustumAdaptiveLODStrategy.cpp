#include "FrustumAdaptiveLODStrategy.h"
#include <algorithm>
#include <cmath>

namespace CADacomb
{
    FrustumAdaptiveLODStrategy::FrustumAdaptiveLODStrategy(double minPixelSize, double maxPixelSize, double targetFillRatio, int maxDepth)
      : minPixelSize(minPixelSize)
      , maxPixelSize(maxPixelSize)
      , targetFillRatio(targetFillRatio)
      , maxDepth(maxDepth)
    {
    }

    FrustumAdaptiveLODStrategy::~FrustumAdaptiveLODStrategy()
    {
    }

    void FrustumAdaptiveLODStrategy::startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t newVertexCapacity, size_t resolution)
    {
        (void)volumeBounds;
        vertexCapacity = newVertexCapacity;
        cameraPos      = glm::dvec3(sceneData.cameraPos);

        totalTreeDepth = 0;
        size_t tempRes = resolution;
        while (tempRes > 1)
        {
            tempRes >>= 1;
            totalTreeDepth++;
        }

        double screenHeight = static_cast<double>(sceneData.resolution.y);
        double cotFovHalf   = std::abs(static_cast<double>(sceneData.projectionMatrix[1][1]));
        projectionFactor    = 0.5 * cotFovHalf * screenHeight;

        glm::dmat4 viewProj = glm::dmat4(sceneData.projectionMatrix) * glm::dmat4(sceneData.viewMatrix);
        extractFrustumPlanes(viewProj);
    }

    LODDecision FrustumAdaptiveLODStrategy::evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)
    {
        lastVertexCount = currentVertexCount;

        int currentDepth = totalTreeDepth - currentLOD;
        if (maxDepth >= 0 && currentDepth > maxDepth)
        {
            return LODDecision::Coarsen;
        }

        if (frustumCullingEnabled && isOutsideFrustum(bound))
        {
            return LODDecision::Coarsen;
        }

        double distance        = std::max(1e-4, bound.distanceTo(cameraPos));
        double projectedPixels = (currentVoxelLength * projectionFactor) / distance;

        double effectiveMax = maxPixelSize * budgetScale;
        double effectiveMin = minPixelSize * budgetScale;

        if (projectedPixels > effectiveMax)
        {
            if (maxDepth >= 0 && currentDepth >= maxDepth)
            {
                return LODDecision::Keep;
            }

            double targetCount = targetFillRatio * static_cast<double>(vertexCapacity);
            if (currentVertexCount >= vertexCapacity || currentVertexCount >= static_cast<size_t>(targetCount * 1.05))
            {
                return LODDecision::Keep;
            }

            return LODDecision::Refine;
        }

        if (projectedPixels < effectiveMin)
        {
            return LODDecision::Coarsen;
        }

        return LODDecision::Keep;
    }

    void FrustumAdaptiveLODStrategy::endFrame()
    {
        if (vertexCapacity == 0)
        {
            return;
        }

        double targetCount = targetFillRatio * static_cast<double>(vertexCapacity);
        double upperLimit  = targetCount * 1.05;
        double lowerLimit  = targetCount * 0.80;
        double current     = static_cast<double>(lastVertexCount);

        if (current > upperLimit)
        {
            double ratio = current / targetCount;
            budgetScale  = std::min(10.0, budgetScale * (1.0 + 0.1 * (ratio - 1.0)));
        }
        else if (current < lowerLimit && budgetScale > 1.0)
        {
            budgetScale = std::max(1.0, budgetScale * 0.95);
        }
    }

    double FrustumAdaptiveLODStrategy::getMinPixelSize() const
    {
        return minPixelSize;
    }

    double FrustumAdaptiveLODStrategy::getMaxPixelSize() const
    {
        return maxPixelSize;
    }

    void FrustumAdaptiveLODStrategy::setMinPixelSize(double minPixels)
    {
        minPixelSize = std::max(0.1, minPixels);
        if (maxPixelSize < minPixelSize * 1.5)
        {
            maxPixelSize = minPixelSize * 2.0;
        }
    }

    void FrustumAdaptiveLODStrategy::setMaxPixelSize(double maxPixels)
    {
        maxPixelSize = std::max(minPixelSize * 1.5, maxPixels);
    }

    double FrustumAdaptiveLODStrategy::getTargetFillRatio() const
    {
        return targetFillRatio;
    }

    void FrustumAdaptiveLODStrategy::setTargetFillRatio(double ratio)
    {
        targetFillRatio = std::clamp(ratio, 0.1, 1.0);
    }

    double FrustumAdaptiveLODStrategy::getBudgetScale() const
    {
        return budgetScale;
    }

    void FrustumAdaptiveLODStrategy::setBudgetScale(double scale)
    {
        budgetScale = std::max(0.1, scale);
    }

    int FrustumAdaptiveLODStrategy::getMaxDepth() const
    {
        return maxDepth;
    }

    void FrustumAdaptiveLODStrategy::setMaxDepth(int depth)
    {
        maxDepth = depth;
    }

    int FrustumAdaptiveLODStrategy::getTotalTreeDepth() const
    {
        return totalTreeDepth;
    }

    bool FrustumAdaptiveLODStrategy::isFrustumCullingEnabled() const
    {
        return frustumCullingEnabled;
    }

    void FrustumAdaptiveLODStrategy::setFrustumCullingEnabled(bool enabled)
    {
        frustumCullingEnabled = enabled;
    }

    void FrustumAdaptiveLODStrategy::extractFrustumPlanes(const glm::dmat4& viewProj)
    {
        glm::dvec4 row0(viewProj[0][0], viewProj[1][0], viewProj[2][0], viewProj[3][0]);
        glm::dvec4 row1(viewProj[0][1], viewProj[1][1], viewProj[2][1], viewProj[3][1]);
        glm::dvec4 row2(viewProj[0][2], viewProj[1][2], viewProj[2][2], viewProj[3][2]);
        glm::dvec4 row3(viewProj[0][3], viewProj[1][3], viewProj[2][3], viewProj[3][3]);

        frustumPlanes[0] = row3 + row0;
        frustumPlanes[1] = row3 - row0;
        frustumPlanes[2] = row3 + row1;
        frustumPlanes[3] = row3 - row1;
        frustumPlanes[4] = row3 + row2;
        frustumPlanes[5] = row3 - row2;

        for (int i = 0; i < 6; i++)
        {
            glm::dvec3 normal(frustumPlanes[i]);
            double     len = glm::length(normal);
            if (len > 1e-8)
            {
                frustumPlanes[i] /= len;
            }
        }
    }

    bool FrustumAdaptiveLODStrategy::isOutsideFrustum(const AABB3& bound) const
    {
        const glm::dvec3& minP = bound.getMin();
        const glm::dvec3& maxP = bound.getMax();

        for (int i = 0; i < 6; i++)
        {
            const glm::dvec4& plane = frustumPlanes[i];
            glm::dvec3        pVertex((plane.x > 0.0) ? maxP.x : minP.x, (plane.y > 0.0) ? maxP.y : minP.y, (plane.z > 0.0) ? maxP.z : minP.z);

            if (glm::dot(glm::dvec3(plane), pVertex) + plane.w < 0.0)
            {
                return true;
            }
        }

        return false;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace CADacomb
{
    TEST_CASE("FrustumAdaptiveLODStrategy/FrustumCulling", "[FrustumAdaptiveLODStrategy]")
    {
        FrustumAdaptiveLODStrategy strategy(2.0, 5.0, 0.9, -1);

        SceneData sceneData{};
        sceneData.resolution       = glm::dvec2(1920.0, 1080.0);
        sceneData.projectionMatrix = glm::perspective(glm::radians(60.0), 16.0 / 9.0, 0.1, 100.0);
        sceneData.viewMatrix       = glm::lookAt(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, -1.0), glm::dvec3(0.0, 1.0, 0.0));
        sceneData.cameraPos        = glm::dvec4(0.0, 0.0, 0.0, 1.0);

        AABB3 volumeBounds(glm::dvec3(-50.0), glm::dvec3(50.0));
        strategy.startFrame(sceneData, volumeBounds, 10000, 1024);

        AABB3 inFrontBox(glm::dvec3(-1.0, -1.0, -10.0), glm::dvec3(1.0, 1.0, -8.0));
        AABB3 behindBox(glm::dvec3(-1.0, -1.0, 5.0), glm::dvec3(1.0, 1.0, 7.0));
        AABB3 farOffBox(glm::dvec3(200.0, 200.0, -10.0), glm::dvec3(210.0, 210.0, -8.0));

        CHECK(strategy.evaluate(behindBox, 0.5, 2, 1000) == LODDecision::Coarsen);
        CHECK(strategy.evaluate(farOffBox, 0.5, 2, 1000) == LODDecision::Coarsen);
        CHECK(strategy.evaluate(inFrontBox, 0.5, 2, 1000) != LODDecision::Coarsen);

        strategy.endFrame();
    }

    TEST_CASE("FrustumAdaptiveLODStrategy/MaxDepthConstraint", "[FrustumAdaptiveLODStrategy]")
    {
        FrustumAdaptiveLODStrategy strategy(2.0, 5.0, 0.9, 8);

        SceneData sceneData{};
        sceneData.resolution       = glm::dvec2(1920.0, 1080.0);
        sceneData.projectionMatrix = glm::perspective(glm::radians(60.0), 16.0 / 9.0, 0.1, 100.0);
        sceneData.viewMatrix       = glm::lookAt(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, -1.0), glm::dvec3(0.0, 1.0, 0.0));
        sceneData.cameraPos        = glm::dvec4(0.0, 0.0, 0.0, 1.0);

        AABB3 volumeBounds(glm::dvec3(-50.0), glm::dvec3(50.0));
        strategy.startFrame(sceneData, volumeBounds, 10000, 1024);

        AABB3 inFrontBox(glm::dvec3(-0.1, -0.1, -1.0), glm::dvec3(0.1, 0.1, -0.8));

        CHECK(strategy.evaluate(inFrontBox, 1.0, 0, 1000) == LODDecision::Coarsen);
        CHECK(strategy.evaluate(inFrontBox, 1.0, 1, 1000) == LODDecision::Coarsen);
        CHECK(strategy.evaluate(inFrontBox, 1.0, 2, 1000) == LODDecision::Keep);
        CHECK(strategy.evaluate(inFrontBox, 1.0, 4, 1000) == LODDecision::Refine);

        strategy.endFrame();
    }
}
#endif
