#include "CADacombRender/SVORaycast/TetraProxy/GPUTetrahedron.h"

namespace CADacomb::Render
{
    GPUTetrahedron::GPUTetrahedron() {}

    GPUTetrahedron::GPUTetrahedron(const glm::uvec4& vertexIndicesInput, const glm::ivec4& neighborTetsInput)
      : vertexIndices(vertexIndicesInput)
      , neighborTets(neighborTetsInput)
    {
    }
}
