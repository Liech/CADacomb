#pragma once

#include "CADacomb/CAD/Interface/CompSolid.h"
#include <memory>
#include <vector>

class SmBrep;

namespace CADacomb::CAD
{
    class CompSolidSm : public CompSolid
    {
      public:
        CompSolidSm(const std::vector<std::shared_ptr<SmBrep>>& brepsInput);
        virtual ~CompSolidSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        std::vector<std::unique_ptr<Solid>> getSolids() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        const std::vector<std::shared_ptr<SmBrep>>& getBreps() const;

      private:
        std::vector<std::shared_ptr<SmBrep>> breps;
    };
}
