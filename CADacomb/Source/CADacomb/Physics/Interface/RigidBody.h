#pragma once

#include <glm/glm.hpp>

namespace CADacomb::Physics
{
    // Abstract interface for a simulated rigid body in a physics world.
    class RigidBody
    {
      public:
        RigidBody();
        virtual ~RigidBody();

        virtual bool       isStatic() const                                    = 0;
        virtual glm::dmat4 getTransformation() const                           = 0;
        virtual void       setTransformation(const glm::dmat4& transformation) = 0;
    };
}
