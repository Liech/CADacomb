#pragma once

#include "CADacomb/CAD/Interface/Compound.h"
#include <memory>
#include <vector>

class SmBrep;

namespace CADacomb::CAD
{
    class CompoundSm : public Compound
    {
      public:
        CompoundSm(const std::vector<std::shared_ptr<SmBrep>>& brepsInput);
        virtual ~CompoundSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        void transform(const glm::dmat4& transformationMatrix) override;
        void translate(const glm::dvec3& translationVector) override;
        void rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians) override;
        void scale(const glm::dvec3& centerPoint, double scaleFactor) override;

        std::vector<std::unique_ptr<Shape>> getShapes() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        const std::vector<std::shared_ptr<SmBrep>>& getBreps() const;

      private:
        std::vector<std::shared_ptr<SmBrep>> breps;
    };
}
