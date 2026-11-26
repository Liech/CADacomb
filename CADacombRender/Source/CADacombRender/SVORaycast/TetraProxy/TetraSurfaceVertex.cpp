#include "CADacombRender/SVORaycast/TetraProxy/TetraSurfaceVertex.h"

namespace CADacomb::Render
{
    TetraSurfaceVertex::TetraSurfaceVertex() {}

    TetraSurfaceVertex::TetraSurfaceVertex(uint32_t vertexIndexInput, uint32_t tetIndexInput)
      : vertexIndex(vertexIndexInput)
      , tetIndex(tetIndexInput)
    {
    }
}
