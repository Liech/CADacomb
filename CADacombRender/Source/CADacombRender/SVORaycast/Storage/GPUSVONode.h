#pragma once

#include <cstdint>

namespace CADacomb::Render
{
    // Packed 8-byte GPU SVO node layout matching Slang GPUSVONode (24-bit materialId + 8-bit child validMask).
    struct GPUSVONode
    {
      public:
        GPUSVONode();
        GPUSVONode(uint32_t childPointerInput, uint32_t materialIdInput, uint32_t validMaskInput);

        uint32_t getMaterialId() const;
        uint32_t getValidMask() const;

        uint32_t childPointer = 0xFFFFFFFF;
        uint32_t payload      = 0;
    };
}
