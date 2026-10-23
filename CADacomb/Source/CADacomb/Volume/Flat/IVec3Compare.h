#pragma once

#include <glm/glm.hpp>

namespace CADacomb
{
    struct IVec3Compare
    {
      public:
        IVec3Compare();
        virtual ~IVec3Compare();

        bool operator()(const glm::ivec3& a, const glm::ivec3& b) const;
    };
}
