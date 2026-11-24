#pragma once

#include <glm/glm.hpp>

namespace CADacomb::Render
{
    // Packed 32-byte GPU tetrahedron topology matching Slang GPUTetrahedron (4 vertices + 4 neighbor tetra indices).
    struct GPUTetrahedron
    {
      public:
        GPUTetrahedron();
        GPUTetrahedron(const glm::uvec4& vertexIndicesInput, const glm::ivec4& neighborTetsInput);

        glm::uvec4 vertexIndices{ 0u };
        glm::ivec4 neighborTets{ -1 };
    };
}
