#pragma once

#include "Face.h"
#include "Shape.h"
#include <memory>
#include <vector>

namespace CADacomb::CAD
{
    class Shell : public Shape
    {
      public:
        Shell();
        virtual ~Shell();

        virtual bool isClosed() const = 0;

        virtual std::vector<std::unique_ptr<Face>> getFaces() const = 0;
    };
}
