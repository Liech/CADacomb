#pragma once

#include <cstddef>

namespace CADacomb
{
    // Triangle entry used during quadric error metric mesh simplification.
    struct SimplificationTriangle
    {
      public:
        SimplificationTriangle();
        virtual ~SimplificationTriangle();

        size_t v[3]    = { 0, 0, 0 };
        bool   deleted = false;
    };
}
