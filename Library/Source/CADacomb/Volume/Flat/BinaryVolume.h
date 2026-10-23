#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace Library
{
    class BinaryVolume
    {
      public:
        BinaryVolume();
        virtual ~BinaryVolume();

        glm::u64vec3      dimension{ 0 };
        glm::dvec3        origin{ 0.0 };
        glm::dvec3        size{ 0.0 };
        std::vector<bool> data;
    };
}
