#pragma once

#include "LODStrategy.h"

namespace CADacomb
{
    class ScreenSpaceLODStrategy : public LODStrategy
    {
      public:
        ScreenSpaceLODStrategy(double minPixelSize = 2.0, double maxPixelSize = 4.0);
        virtual ~ScreenSpaceLODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) override;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount) override;
        virtual void        endFrame() override;

        double getMinPixelSize() const;
        double getMaxPixelSize() const;
        void   setMinPixelSize(double minPixels);
        void   setMaxPixelSize(double maxPixels);

      private:
        glm::dvec3 cameraPos{ 0.0 };
        double     projectionFactor = 0.0;
        double     minPixelSize     = 2.0;
        double     maxPixelSize     = 4.0;
    };
}
