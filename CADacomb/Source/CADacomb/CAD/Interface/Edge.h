#pragma once

#include "CurveType.h"
#include "Orientation.h"
#include "Shape.h"
#include "Vertex.h"
#include <memory>

namespace CADacomb::CAD
{
    class Edge : public Shape
    {
      public:
        Edge();
        virtual ~Edge();

        virtual CurveType   getCurveType() const   = 0;
        virtual Orientation getOrientation() const = 0;

        virtual std::unique_ptr<Vertex> getStart() const = 0;
        virtual std::unique_ptr<Vertex> getEnd() const   = 0;
    };
}
