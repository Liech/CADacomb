#pragma once

#include "CADacomb/SDF/Sdf.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <cstdint>
#include <memory>
#include <string>

namespace CADacomb
{
    // Converts a CADacomb::Sdf into an indexed Triangulation via SurfaceNets on the signed distance field.
    class SdfToTriangulationConverter
    {
      public:
        SdfToTriangulationConverter();
        virtual ~SdfToTriangulationConverter();

        static std::unique_ptr<Triangulation> convert(const Sdf& sdf, uint32_t resolution, std::string& outError);
    };
}
