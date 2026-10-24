#pragma once

#include "CADacomb/CAD/Interface/Edge.h"
#include <memory>

class SmBrep;
class SmEdge;

namespace CADacomb::CAD
{
    class EdgeSm : public Edge
    {
      public:
        EdgeSm(const std::shared_ptr<SmBrep>& brepInput, SmEdge* edgeInput, Orientation orientationInput);
        virtual ~EdgeSm();

        ShapeType getShapeType() const override;
        AABB3     getAABB() const override;

        CurveType   getCurveType() const override;
        Orientation getOrientation() const override;

        std::unique_ptr<Vertex>             getStart() const override;
        std::unique_ptr<Vertex>             getEnd() const override;
        std::vector<std::unique_ptr<Shape>> getChildren() const override;

        SmEdge* getEdge() const;

      private:
        std::shared_ptr<SmBrep> brep;
        SmEdge*                 edge;
        Orientation             orientation;
    };
}
