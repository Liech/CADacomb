#pragma once

#include "CADacomb/Triangle/Triangulation.h"
#include "Shape.h"
#include "Solid.h"
#include "TessellationOptions.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace CADacomb::CAD
{
    class Kernel
    {
      public:
        Kernel();
        virtual ~Kernel();

        virtual std::unique_ptr<Solid> createBox(const glm::dvec3& minCorner, const glm::dvec3& dimensions) const       = 0;
        virtual std::unique_ptr<Solid> createCylinder(const glm::dvec3& baseCenter, double radius, double height) const = 0;

        virtual std::unique_ptr<Shape> booleanUnion(const Shape& shapeA, const Shape& shapeB) const        = 0;
        virtual std::unique_ptr<Shape> booleanDifference(const Shape& shapeA, const Shape& shapeB) const   = 0;
        virtual std::unique_ptr<Shape> booleanIntersection(const Shape& shapeA, const Shape& shapeB) const = 0;

        virtual std::unique_ptr<Triangulation> tessellate(const Shape& shape, const TessellationOptions& options) const = 0;

        virtual std::vector<std::unique_ptr<Shape>> loadStepFile(const std::string& filePath) const                                                     = 0;
        virtual void                                saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const = 0;
    };
}
