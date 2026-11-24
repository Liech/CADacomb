#pragma once

#include <cstdint>

namespace CADacomb::Render
{
    // Static boundary-triangle vertex referencing a cage vertex index and its owning tetrahedron index.
    struct TetraSurfaceVertex
    {
      public:
        TetraSurfaceVertex();
        TetraSurfaceVertex(uint32_t vertexIndexInput, uint32_t tetIndexInput);

        uint32_t vertexIndex = 0u;
        uint32_t tetIndex    = 0u;
    };
}
