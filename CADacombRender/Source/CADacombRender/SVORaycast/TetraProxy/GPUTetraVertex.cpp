#include "CADacombRender/SVORaycast/TetraProxy/GPUTetraVertex.h"

namespace CADacomb::Render
{
    GPUTetraVertex::GPUTetraVertex() {}

    GPUTetraVertex::GPUTetraVertex(const glm::vec3& restVoxelPosInput, const glm::vec3& deformedLocalPosInput)
      : restVoxelPos(restVoxelPosInput, 0.0f)
      , deformedLocalPos(deformedLocalPosInput, 0.0f)
    {
    }
}
