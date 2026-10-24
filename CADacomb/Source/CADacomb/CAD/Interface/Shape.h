#pragma once

#include "CADacomb/Volume/AABB3.h"
#include "ShapeType.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Shape
    {
      public:
        Shape();
        virtual ~Shape();

        virtual ShapeType getShapeType() const = 0;
        virtual AABB3     getAABB() const      = 0;

        virtual void transform(const glm::dmat4& transformationMatrix)                                          = 0;
        virtual void translate(const glm::dvec3& translationVector)                                             = 0;
        virtual void rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians) = 0;
        virtual void scale(const glm::dvec3& centerPoint, double scaleFactor)                                   = 0;

        virtual std::vector<std::unique_ptr<Shape>> getChildren() const = 0;
    };
}
