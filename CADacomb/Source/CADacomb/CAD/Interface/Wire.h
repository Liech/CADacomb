#pragma once

#include "Edge.h"
#include "Shape.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Wire : public Shape
    {
      public:
        Wire();
        virtual ~Wire();

        virtual bool isClosed() const    = 0;
        virtual bool isOuterWire() const = 0;

        virtual std::vector<std::unique_ptr<Edge>> getEdges() const = 0;
    };
}
