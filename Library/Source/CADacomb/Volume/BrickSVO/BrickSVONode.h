#pragma once

#include <cstdint>

namespace Library::Volume
{
    template<typename T>
    struct BrickSVONode
    {
        uint32_t childPointer = 0xFFFFFFFF;
        uint32_t brickPointer = 0xFFFFFFFF;
        T        value{};
    };
}
