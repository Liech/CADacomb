#pragma once

#include <cstdint>

namespace Library::Volume
{
    template<typename T>
    struct SVONode
    {
        uint32_t childPointer = 0xFFFFFFFF;
        T        value{};
    };
}
