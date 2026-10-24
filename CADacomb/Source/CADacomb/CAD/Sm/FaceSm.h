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

        double      getSurfaceArea() const override;
        SurfaceType getSurfaceType() const override;
        Orientation getOrientation() const override;

        std::vector<std::unique_ptr<Wire>>  getWires() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmFace* getFace() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmFace*                 face;
        Orientation             orientation;
    };
}
