#pragma once

#include "CADacomb/Volume/AABB3.h"
#include "ShapeType.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Shape
    {
      public:
        Shape();
        virtual ~Shape();

        virtual ShapeType getShapeType() const = 0;
        virtual AABB3     getAABB() const      = 0;

        virtual std::vector<std::unique_ptr<Shape>> getChildren() const = 0;
    };
}
