#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace Library::Volume
{
    template<typename T>
    struct SVODualNode
    {
        uint32_t   nodeIndex = 0xFFFFFFFF;
        glm::ivec3 offset{ 0 };
        uint32_t   size  = 0;
        int        depth = 0;
        T          value{};
        bool       isLeaf    = true;
        bool       isVirtual = false;
    };
}
