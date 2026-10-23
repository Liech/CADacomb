#pragma once

#include "LODStrategy.h"

namespace CADacomb
{
    class FixedLODStrategy : public LODStrategy
    {
      public:
        FixedLODStrategy(LODDecision fixedDecision = LODDecision::Keep);
        virtual ~FixedLODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) override;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount) override;
        virtual void        endFrame() override;

        LODDecision getDecision() const;
        void        setDecision(LODDecision newDecision);

      private:
        LODDecision decision = LODDecision::Keep;
    };
}
