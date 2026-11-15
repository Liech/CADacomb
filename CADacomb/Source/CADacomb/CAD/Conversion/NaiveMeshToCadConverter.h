#pragma once

#include <memory>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    class Kernel;
    class Shape;

    // Converts each triangle of a Triangulation into an individual planar CAD face and stitches them.
    class NaiveMeshToCadConverter
    {
      public:
        NaiveMeshToCadConverter();
        virtual ~NaiveMeshToCadConverter();

        std::unique_ptr<Shape> convert(const Kernel& kernel, const Triangulation& mesh, double stitchTolerance, bool tryMakeSolid) const;
    };
}
