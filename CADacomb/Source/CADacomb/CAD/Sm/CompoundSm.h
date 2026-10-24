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

        std::vector<std::unique_ptr<Shape>> getShapes() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        const std::vector<std::shared_ptr<SmBrep>>& getBreps() const;

      private:
        std::vector<std::shared_ptr<SmBrep>> breps;
    };
}
