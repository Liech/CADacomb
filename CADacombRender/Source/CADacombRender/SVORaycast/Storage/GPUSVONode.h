#pragma once

#include <cstdint>

namespace CADacomb::Render
{
    // Packed 8-byte GPU SVO node layout matching Slang GPUSVONode.
    struct GPUSVONode
    {
      public:
        GPUSVONode();
        GPUSVONode(uint32_t childPointerInput, uint32_t valueInput);

        uint32_t childPointer = 0xFFFFFFFF;
        uint32_t value        = 0;
    };
}
