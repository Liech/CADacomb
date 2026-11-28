#pragma once

#include "CADacomb/SDF/Sdf.h"
#include "CADacomb/Volume/SVO/SVO.h"

#include <cstdint>
#include <memory>
#include <string>

namespace CADacomb
{
    // Converts a CADacomb::Sdf into a boolean SVO using top-down octree pruning.
    class SdfToSVOConverter
    {
      public:
        SdfToSVOConverter();
        virtual ~SdfToSVOConverter();

        static std::unique_ptr<SVO<bool>> convert(const Sdf& sdf, uint32_t dimension, std::string& outError);
    };
}
