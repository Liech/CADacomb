#pragma once

#include "Orientation.h"
#include "Shape.h"
#include "SurfaceType.h"
#include "Wire.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Face : public Shape
    {
      public:
        Face();
        virtual ~Face();

        virtual double      getSurfaceArea() const = 0;
        virtual SurfaceType getSurfaceType() const = 0;
        virtual Orientation getOrientation() const = 0;

        virtual std::vector<std::unique_ptr<Wire>> getWires() const = 0;
    };
}
