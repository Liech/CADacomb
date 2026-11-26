#pragma once

#include <glm/glm.hpp>

namespace CADacomb::Render
{
    // Packed 32-byte GPU tetrahedron cage vertex matching Slang GPUTetraVertex.
    struct GPUTetraVertex
    {
      public:
        GPUTetraVertex();
        GPUTetraVertex(const glm::vec3& restVoxelPosInput, const glm::vec3& deformedLocalPosInput);

        glm::vec4 restVoxelPos{ 0.0f };
        glm::vec4 deformedLocalPos{ 0.0f };
    };
}
