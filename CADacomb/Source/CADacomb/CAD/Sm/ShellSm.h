#pragma once

#include "CADacomb/CAD/Interface/Shell.h"
#include <memory>

class SmBrep;
class SmShell;

namespace CADacomb::CAD
{
    class ShellSm : public Shell
    {
      public:
        ShellSm(const std::shared_ptr<SmBrep>& brepInput, SmShell* shellInput);
        virtual ~ShellSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        void transform(const glm::dmat4& transformationMatrix) override;
        void translate(const glm::dvec3& translationVector) override;
        void rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians) override;
        void scale(const glm::dvec3& centerPoint, double scaleFactor) override;

        bool isClosed() const override;

        std::vector<std::unique_ptr<Face>>  getFaces() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmShell* getShell() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmShell*                shell;
    };
}
