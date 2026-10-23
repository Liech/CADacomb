#pragma once

#include <cstdint>

namespace CADacomb
{
    template<typename T>
    struct SVONode
    {
        uint32_t childPointer = 0xFFFFFFFF;
        T        value{};
    };
}
