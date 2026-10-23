#pragma once

#include <glm/glm.hpp>

namespace CADacomb
{
    struct SVOMesherOptions
    {
      public:
        SVOMesherOptions();
        virtual ~SVOMesherOptions();

        glm::dvec3 focalPoint      = glm::dvec3(0.0);
        glm::dvec3 origin          = glm::dvec3(0.0);
        glm::dvec3 size            = glm::dvec3(1.0);
        double     lodNearDistance = 1.0;
        double     lodFarDistance  = 10.0;
        int        chunkSize       = 32;
        int        maxResolution   = 1024;
        int        minLOD          = 0;
        int        maxLOD          = 3;
        bool       adaptive        = false;
    };
}
