#pragma once

#include "Shape.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Compound : public Shape
    {
      public:
        Compound();
        virtual ~Compound();

        virtual std::vector<std::unique_ptr<Shape>> getShapes() const = 0;
    };
}
