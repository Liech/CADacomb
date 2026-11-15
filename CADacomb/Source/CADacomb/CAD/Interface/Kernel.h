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

        virtual std::unique_ptr<Shape> createPlanarFace(const glm::dvec3& planeOrigin, const glm::dvec3& planeNormal, const std::vector<std::vector<glm::dvec3>>& loops) const          = 0;
        virtual std::unique_ptr<Shape> createBSplineFace(size_t numRows, size_t numCols, const std::vector<glm::dvec3>& controlPoints, const std::vector<std::vector<glm::dvec3>>& loops) const = 0;
        virtual std::unique_ptr<Shape> createCylindricalFace(const glm::dvec3& origin, const glm::dvec3& axis, double radius, double height, const std::vector<std::vector<glm::dvec3>>& loops) const = 0;
        virtual std::unique_ptr<Shape> createConicalFace(const glm::dvec3& origin, const glm::dvec3& axis, double botRadius, double topRadius, double height, const std::vector<std::vector<glm::dvec3>>& loops) const = 0;
        virtual std::unique_ptr<Shape> createSphericalFace(const glm::dvec3& center, double radius, const std::vector<std::vector<glm::dvec3>>& loops) const = 0;
        virtual std::unique_ptr<Shape> mergeShapes(const std::vector<const Shape*>& shapes) const                                                                               = 0;
        virtual std::unique_ptr<Shape> stitch(const Shape& shape, double tolerance, bool tryMakeSolid) const                                                                    = 0;
        virtual void                   unifyNormals(Shape& shape) const                                                                                                         = 0;

        virtual std::unique_ptr<Shape> booleanUnion(const Shape& shapeA, const Shape& shapeB) const        = 0;
        virtual std::unique_ptr<Shape> booleanDifference(const Shape& shapeA, const Shape& shapeB) const   = 0;
        virtual std::unique_ptr<Shape> booleanIntersection(const Shape& shapeA, const Shape& shapeB) const = 0;

        virtual std::unique_ptr<Triangulation> tessellate(const Shape& shape, const TessellationOptions& options) const = 0;

        virtual std::vector<std::unique_ptr<Shape>> loadStepFile(const std::string& filePath) const                                                     = 0;
        virtual void                                saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const = 0;
    };
}
