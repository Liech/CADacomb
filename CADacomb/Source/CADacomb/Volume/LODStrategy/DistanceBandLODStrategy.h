#pragma once

#include "LODStrategy.h"

namespace CADacomb
{
    class DistanceBandLODStrategy : public LODStrategy
    {
      public:
        DistanceBandLODStrategy(double nearDistance = 10.0, double farDistance = 50.0);
        virtual ~DistanceBandLODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) override;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount) override;
        virtual void        endFrame() override;

        double getNearDistance() const;
        double getFarDistance() const;
        void   setNearDistance(double nearDist);
        void   setFarDistance(double farDist);

      private:
        glm::dvec3 cameraPos{ 0.0 };
        double     nearDistance = 10.0;
        double     farDistance  = 50.0;
    };
}
