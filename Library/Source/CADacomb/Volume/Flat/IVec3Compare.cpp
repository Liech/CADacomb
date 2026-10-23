#include "IVec3Compare.h"

namespace Library
{
    IVec3Compare::IVec3Compare() {}

    IVec3Compare::~IVec3Compare() {}

    bool IVec3Compare::operator()(const glm::ivec3& a, const glm::ivec3& b) const
    {
        if (a.x != b.x)
        {
            return a.x < b.x;
        }
        if (a.y != b.y)
        {
            return a.y < b.y;
        }
        return a.z < b.z;
    }
}
