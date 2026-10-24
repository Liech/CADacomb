#pragma once

#include "CADacomb/CAD/Interface/Kernel.h"

class SmBrep;

namespace CADacomb::CAD
{
    class KernelSm : public Kernel
    {
      public:
        KernelSm();
        virtual ~KernelSm();

        std::unique_ptr<Solid> createBox(const glm::dvec3& minCorner, const glm::dvec3& dimensions) const override;
        std::unique_ptr<Solid> createCylinder(const glm::dvec3& baseCenter, double radius, double height) const override;

        std::unique_ptr<Shape> booleanUnion(const Shape& shapeA, const Shape& shapeB) const override;
        std::unique_ptr<Shape> booleanDifference(const Shape& shapeA, const Shape& shapeB) const override;
        std::unique_ptr<Shape> booleanIntersection(const Shape& shapeA, const Shape& shapeB) const override;

        std::unique_ptr<Triangulation> tessellate(const Shape& shape, const TessellationOptions& options) const override;

        std::vector<std::unique_ptr<Shape>> loadStepFile(const std::string& filePath) const override;
        void                                saveStepFile(const Shape& shape, const std::string& filePath, const std::string& productName) const override;

      private:
        SmBrep* copyBrepFromShape(const Shape& shape) const;
    };
}
