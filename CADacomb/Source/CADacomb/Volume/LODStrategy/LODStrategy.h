#pragma once

#include "CADacomb/Volume/AABB3.h"
#include "LODDecision.h"
#include "SceneData.h"
#include <cstddef>

namespace CADacomb
{
    class LODStrategy
    {
      public:
        LODStrategy();
        virtual ~LODStrategy();

        virtual void        startFrame(const SceneData& sceneData, const AABB3& volumeBounds, size_t vertexCapacity, size_t resolution) = 0;
        virtual LODDecision evaluate(const AABB3& bound, double currentVoxelLength, int currentLOD, size_t currentVertexCount)          = 0;
        virtual void        endFrame()                                                                                                 = 0;
    };
}
