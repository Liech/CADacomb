#pragma once

#include <glm/glm.hpp>
#include <memory>

class SmBrep;

namespace CADacomb::CAD
{
    class BrepTransformSm
    {
      public:
        BrepTransformSm();
        virtual ~BrepTransformSm();

        static std::shared_ptr<SmBrep> cloneBrep(const std::shared_ptr<SmBrep>& brep);
        static void                    transform(SmBrep* brep, const glm::dmat4& transformationMatrix);
        static void translate(SmBrep* brep, const glm::dvec3& translationVector);
        static void rotate(SmBrep* brep, const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians);
        static void scale(SmBrep* brep, const glm::dvec3& centerPoint, double scaleFactor);
    };
}
