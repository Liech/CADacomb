#pragma once

#include "Shape.h"
#include "Solid.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class CompSolid : public Shape
    {
      public:
        CompSolid();
        virtual ~CompSolid();

        virtual std::vector<std::unique_ptr<Solid>> getSolids() const = 0;
    };
}
