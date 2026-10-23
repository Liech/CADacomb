#pragma once

#include <glm/glm.hpp>

namespace Library::Volume
{
    struct SVOBlockyOptions
    {
      public:
        SVOBlockyOptions();
        virtual ~SVOBlockyOptions();

        int        targetDepth     = -1;
        glm::dvec3 origin          = glm::dvec3(0.0);
        glm::dvec3 size            = glm::dvec3(1.0);
        bool       adaptive        = false;
        glm::dvec3 focalPoint      = glm::dvec3(0.0);
        double     lodDistance     = 1.0;
        int        minDepth        = 1;
        int        maxDepth        = -1;
        double     lodNearDistance = 0.0;
        double     lodFarDistance  = 0.0;
    };
}
