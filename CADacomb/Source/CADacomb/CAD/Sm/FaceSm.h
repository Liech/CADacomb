#pragma once

#include "CADacomb/CAD/Interface/Face.h"
#include <memory>

class SmBrep;
class SmFace;

namespace CADacomb::CAD
{
    class FaceSm : public Face
    {
      public:
        FaceSm(const std::shared_ptr<SmBrep>& brepInput, SmFace* faceInput, Orientation orientationInput);
        virtual ~FaceSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        void transform(const glm::dmat4& transformationMatrix) override;
        void translate(const glm::dvec3& translationVector) override;
        void rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians) override;
        void scale(const glm::dvec3& centerPoint, double scaleFactor) override;

        double      getSurfaceArea() const override;
        SurfaceType getSurfaceType() const override;
        Orientation getOrientation() const override;

        std::vector<std::unique_ptr<Wire>>  getWires() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        std::pair<glm::dvec2, glm::dvec2> getUVBounds() const override;
        glm::dvec3                        evaluatePoint(double uParameter, double vParameter) const override;
        glm::dvec3                        evaluateNormal(double uParameter, double vParameter) const override;
        std::pair<glm::dvec3, glm::dvec3> evaluateTangents(double uParameter, double vParameter) const override;

        SmFace* getFace() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmFace*                 face;
        Orientation             orientation;
    };
}
