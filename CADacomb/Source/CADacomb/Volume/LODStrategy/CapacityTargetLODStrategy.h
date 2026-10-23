#pragma once

#include "LODStrategy.h"

namespace CADacomb
{
    class CapacityTargetLODStrategy : public LODStrategy
    {
      public:
        CapacityTargetLODStrategy(double targetFillRatio = 0.9, double errorMargin = 0.05, double initialLogScale = 500.0);
        virtual ~CapacityTargetLODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) override;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount) override;
        virtual void        endFrame() override;

        double getTargetFillRatio() const;
        void   setTargetFillRatio(double ratio);

        double getErrorMargin() const;
        void   setErrorMargin(double margin);

        double getLogScale() const;
        void   setLogScale(double scale);

      private:
        glm::dvec3 cameraPos{ 0.0 };
        size_t     vertexCapacity  = 0;
        size_t     lastVertexCount = 0;
        double     targetFillRatio = 0.9;
        double     errorMargin     = 0.05;
        double     logScale        = 500.0;
        double     refDistance     = 1.0;
    };
}
