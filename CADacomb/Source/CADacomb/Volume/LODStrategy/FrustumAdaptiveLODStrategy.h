#pragma once

#include "LODStrategy.h"
#include <glm/glm.hpp>

namespace CADacomb
{
    class FrustumAdaptiveLODStrategy : public LODStrategy
    {
      public:
        FrustumAdaptiveLODStrategy(double minPixelSize = 2.0, double maxPixelSize = 5.0, double targetFillRatio = 0.9, int maxDepth = -1);
        virtual ~FrustumAdaptiveLODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) override;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount) override;
        virtual void        endFrame() override;

        double getMinPixelSize() const;
        double getMaxPixelSize() const;
        void   setMinPixelSize(double minPixels);
        void   setMaxPixelSize(double maxPixels);

        double getTargetFillRatio() const;
        void   setTargetFillRatio(double ratio);

        double getBudgetScale() const;
        void   setBudgetScale(double scale);

        int  getMaxDepth() const;
        void setMaxDepth(int depth);

        int getTotalTreeDepth() const;

        bool isFrustumCullingEnabled() const;
        void setFrustumCullingEnabled(bool enabled);

      private:
        void extractFrustumPlanes(const glm::dmat4& viewProj);
        bool isOutsideFrustum(const AABB3& bound) const;

        glm::dvec4 frustumPlanes[6]{};
        glm::dvec3 cameraPos{ 0.0 };
        size_t     vertexCapacity        = 0;
        size_t     lastVertexCount       = 0;
        double     projectionFactor      = 0.0;
        double     minPixelSize          = 2.0;
        double     maxPixelSize          = 5.0;
        double     targetFillRatio       = 0.9;
        double     budgetScale           = 1.0;
        int        maxDepth              = -1;
        int        totalTreeDepth        = 0;
        bool       frustumCullingEnabled = true;
    };
}
