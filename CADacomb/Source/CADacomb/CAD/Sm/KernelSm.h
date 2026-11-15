#pragma once

#include "CADacomb/CAD/Interface/Kernel.h"

class SmBrep;
class SmSurface;

namespace CADacomb::CAD
{
    class KernelSm : public Kernel
    {
      public:
        KernelSm();
        virtual ~KernelSm();

        std::unique_ptr<Solid> createBox(const glm::dvec3& minCorner, const glm::dvec3& dimensions) const override;
        std::unique_ptr<Solid> createCylinder(const glm::dvec3& baseCenter, double radius, double height) const override;

        std::unique_ptr<Shape> createPlanarFace(const glm::dvec3& planeOrigin, const glm::dvec3& planeNormal, const std::vector<std::vector<glm::dvec3>>& loops) const override;
        std::unique_ptr<Shape> createBSplineFace(size_t numRows, size_t numCols, const std::vector<glm::dvec3>& controlPoints, const std::vector<std::vector<glm::dvec3>>& loops) const override;
        std::unique_ptr<Shape> createCylindricalFace(const glm::dvec3& origin, const glm::dvec3& axis, double radius, double height, const std::vector<std::vector<glm::dvec3>>& loops) const override;
        std::unique_ptr<Shape> createConicalFace(const glm::dvec3& origin, const glm::dvec3& axis, double botRadius, double topRadius, double height, const std::vector<std::vector<glm::dvec3>>& loops) const override;
        std::unique_ptr<Shape> createSphericalFace(const glm::dvec3& center, double radius, const std::vector<std::vector<glm::dvec3>>& loops) const override;
        std::unique_ptr<Shape> mergeShapes(const std::vector<const Shape*>& shapes) const override;
        std::unique_ptr<Shape> stitch(const Shape& shape, double tolerance, bool tryMakeSolid) const override;
        void                   unifyNormals(Shape& shape) const override;

        std::unique_ptr<Shape> booleanUnion(const Shape& shapeA, const Shape& shapeB) const override;
        std::unique_ptr<Shape> booleanDifference(const Shape& shapeA, const Shape& shapeB) const override;
        std::unique_ptr<Shape> booleanIntersection(const Shape& shapeA, const Shape& shapeB) const override;

        std::unique_ptr<Triangulation> tessellate(const Shape& shape, const TessellationOptions& options) const override;

        std::vector<std::unique_ptr<Shape>> loadStepFile(const std::string& filePath) const override;
        void                                saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const override;

      private:
        SmBrep*                copyBrepFromShape(const Shape& shape) const;
        std::unique_ptr<Shape> createFaceFromUVCurves(SmSurface* surface, const std::vector<std::vector<glm::dvec3>>& loops) const;
        std::unique_ptr<Shape> trimSurfaceWith3dCurves(SmSurface* surface, const std::vector<std::vector<glm::dvec3>>& loops) const;
    };
}
