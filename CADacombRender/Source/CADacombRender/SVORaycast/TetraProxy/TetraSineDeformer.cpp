#include "CADacombRender/SVORaycast/TetraProxy/TetraSineDeformer.h"

#include <cmath>

namespace CADacomb::Render
{
    TetraSineDeformer::TetraSineDeformer() {}

    TetraSineDeformer::~TetraSineDeformer() {}

    void TetraSineDeformer::deform(std::vector<GPUTetraVertex>& vertices,
                                   const CADacomb::AABB3&       volumeBounds,
                                   uint32_t                     svoDimension,
                                   float                        timeSeconds,
                                   float                        amplitudeFactor)
    {
        if (svoDimension == 0u)
        {
            return;
        }

        glm::vec3 bMin   = glm::vec3(volumeBounds.getMin());
        glm::vec3 bSize  = glm::vec3(volumeBounds.getSize());
        float     invDim = 1.0f / static_cast<float>(svoDimension);

        for (GPUTetraVertex& vertex : vertices)
        {
            glm::vec3 uvw      = glm::vec3(vertex.restVoxelPos) * invDim;
            glm::vec3 basePos  = bMin + uvw * bSize;
            float     heightW  = uvw.y;
            float     phaseX   = timeSeconds * 2.2f + uvw.y * 3.14159f;
            float     phaseZ   = timeSeconds * 1.7f + uvw.x * 3.14159f;
            float     offsetX  = std::sin(phaseX) * bSize.x * amplitudeFactor * heightW;
            float     offsetZ  = std::cos(phaseZ) * bSize.z * amplitudeFactor * 0.65f * heightW;
            float     offsetY  = std::sin(timeSeconds * 1.5f + (uvw.x + uvw.z) * 3.14159f) * bSize.y * amplitudeFactor * 0.35f;

            vertex.deformedLocalPos = glm::vec4(basePos + glm::vec3(offsetX, offsetY, offsetZ), 0.0f);
        }
    }
}
