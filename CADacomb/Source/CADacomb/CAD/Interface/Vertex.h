#pragma once

#include "Shape.h"
#include <glm/glm.hpp>

namespace CADacomb::CAD
{
    class Vertex : public Shape
    {
      public:
        Vertex();
        virtual ~Vertex();

        virtual glm::dvec3 getPosition() const = 0;
    };
}
