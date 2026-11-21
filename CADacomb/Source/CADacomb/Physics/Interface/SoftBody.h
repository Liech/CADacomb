#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Physics
{
    // Abstract interface for a simulated deformable soft body in a physics world.
    class SoftBody
    {
      public:
        SoftBody();
        virtual ~SoftBody();

        virtual std::vector<glm::dvec3> getDeformedVertices() const                         = 0;
        virtual glm::dmat4              getTransformation() const                           = 0;
        virtual void                    setTransformation(const glm::dmat4& transformation) = 0;
    };
}
