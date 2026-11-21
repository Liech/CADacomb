#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"

#include <glm/glm.hpp>

namespace CADacomb
{
    // Analytical mass, center of mass, and inertia tensor computed directly from solid SVO octree leaves.
    class SVOMassProperties
    {
      public:
        SVOMassProperties();
        SVOMassProperties(double mass, const glm::dvec3& centerOfMass, const glm::dmat3& inertiaTensor);
        virtual ~SVOMassProperties();

        static SVOMassProperties compute(const SVO<bool>& svo, const AABB3& localBounds, double density = 1000.0);

        double            getMass() const;
        const glm::dvec3& getCenterOfMass() const;
        const glm::dmat3& getInertiaTensor() const;

      private:
        double     mass = 1.0;
        glm::dvec3 centerOfMass{ 0.0 };
        glm::dmat3 inertiaTensor{ 1.0 };
    };
}
