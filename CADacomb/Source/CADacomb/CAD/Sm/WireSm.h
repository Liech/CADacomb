#pragma once

#include "CADacomb/CAD/Interface/Wire.h"
#include <memory>

class SmBrep;
class SmLoopuse;

namespace CADacomb::CAD
{
    class WireSm : public Wire
    {
      public:
        WireSm(const std::shared_ptr<SmBrep>& brepInput, SmLoopuse* loopuseInput, bool outerWireInput);
        virtual ~WireSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        bool isClosed() const override;
        bool isOuterWire() const override;

        std::vector<std::unique_ptr<Edge>>  getEdges() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmLoopuse* getLoopuse() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmLoopuse*              loopuse;
        bool                    outerWire;
    };
}
